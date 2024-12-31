#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <image_geometry/pinhole_camera_model.h>
#include <cv_bridge/cv_bridge.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/filters/crop_box.h>  // Include the CropBox filter
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <yolov8_msgs/msg/detection_array.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

// Define the LidarCameraFusionNode class which inherits from rclcpp::Node
class LidarCameraFusionNode : public rclcpp::Node
{
public:
    // Constructor for the node
    LidarCameraFusionNode()
    : Node("lidar_camera_fusion_node"),  // Initialize the node with a name
      tf_buffer_(this->get_clock()),    // Initialize tf2 buffer
      tf_listener_(tf_buffer_)         // Initialize tf2 listener
    {
        // Declare and retrieve parameters for filtering the point cloud
        this->declare_parameter<std::string>("camera_frame", "interceptor/gimbal_camera");
        this->declare_parameter<float>("min_depth", 0.2);
        this->declare_parameter<float>("max_depth", 10.0);
        this->get_parameter("camera_frame", camera_frame_);
        this->get_parameter("min_depth", min_depth_);
        this->get_parameter("max_depth", max_depth_);

        // Log parameters
        RCLCPP_INFO(this->get_logger(), "Loaded Parameters: Camera Frame = %s, MinDepth = %f, MaxDepth = %f",
                    camera_frame_.c_str(), min_depth_, max_depth_);

        // Create message_filters subscribers
        point_cloud_sub_.subscribe(this, "/scan/points");
        image_sub_.subscribe(this, "/interceptor/gimbal_camera");
        detection_sub_.subscribe(this, "/rgb/tracking");

        // Synchronize the subscribers using an ApproximateTime policy
        sync_.reset(new Sync(ApproximateSyncPolicy(10), point_cloud_sub_, image_sub_, detection_sub_));
        sync_->registerCallback(std::bind(&LidarCameraFusionNode::sync_callback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

        // Create a subscription to the camera info
        camera_info_subscriber_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            "/interceptor/gimbal_camera_info", 10, std::bind(&LidarCameraFusionNode::camera_info_callback, this, std::placeholders::_1));

        // Create a publisher for the lidar/camera fusion
        image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/image_lidar_fusion", 10);

        // Create a publisher for the detected object Pose
        pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/detected_object_pose", 10);

        // Create a publisher for detected object point clouds
        object_point_cloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/detected_object_point_cloud", 10);
    }

private:
    struct BoundingBox {
        double x_min, y_min, x_max, y_max;
        double sum_x = 0, sum_y = 0, sum_z = 0;  // Sum of coordinates
        int count = 0;  // Number of points within the bounding box
        bool valid = false;  // Indicates if the bbox is valid
        int id = -1;  // ID of the bounding box
    };

    // Callback function to process incoming camera info
    void camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
    {
        // Initialize camera model
        camera_model_.fromCameraInfo(msg);
        image_width_ = msg->width;
        image_height_ = msg->height;
    }

    // Synchronized callback function
    void sync_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& point_cloud_msg,
                       const sensor_msgs::msg::Image::ConstSharedPtr& image_msg,
                       const yolov8_msgs::msg::DetectionArray::ConstSharedPtr& detection_msg)
    {
        // Convert ROS point cloud message to PCL point cloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromROSMsg(*point_cloud_msg, *cloud);

        // Apply the CropBox filter to limit points within the specified box
        pcl::CropBox<pcl::PointXYZ> box_filter;
        box_filter.setInputCloud(cloud);
        box_filter.setMin(Eigen::Vector4f(min_depth_, -max_depth_, -max_depth_, 1.0f));  // x: min_depth_ to max_depth_, y: -max_depth_ to +max_depth_, z: -max_depth_ to +max_depth_
        box_filter.setMax(Eigen::Vector4f(max_depth_, max_depth_, max_depth_, 1.0f));
        box_filter.filter(*cloud_filtered);

        // Convert the filtered PCL point cloud back to ROS message format
        sensor_msgs::msg::PointCloud2 filtered_msg;
        pcl::toROSMsg(*cloud_filtered, filtered_msg);
        filtered_msg.header.stamp = point_cloud_msg->header.stamp;
        filtered_msg.header.frame_id = point_cloud_msg->header.frame_id;

        // Check if a valid transformation exists and apply it to the filtered point cloud
        sensor_msgs::msg::PointCloud2 cloud_transformed;
        if (!tf_buffer_.canTransform(camera_frame_, filtered_msg.header.frame_id, filtered_msg.header.stamp, tf2::durationFromSec(1.0)))
        {
            RCLCPP_INFO(this->get_logger(), "No transform available.");
            return;
        }

        tf_buffer_.transform(filtered_msg, cloud_transformed, camera_frame_);

        // Convert the transformed point cloud back to PCL format for easy manipulation
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_camera_frame(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromROSMsg(cloud_transformed, *cloud_camera_frame);

        projected_points_.clear();  // Clear previous points

        // Process YOLO detections
        bounding_boxes.clear();
        for (const auto& detection : detection_msg->detections) {
            BoundingBox bbox;
            bbox.x_min = detection.bbox.center.position.x - detection.bbox.size.x / 2.0;
            bbox.y_min = detection.bbox.center.position.y - detection.bbox.size.y / 2.0;
            bbox.x_max = detection.bbox.center.position.x + detection.bbox.size.x / 2.0;
            bbox.y_max = detection.bbox.center.position.y + detection.bbox.size.y / 2.0;
            bbox.valid = true;
            try {
                bbox.id = std::stoi(detection.id);  // Convert string ID to integer
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "Failed to convert detection ID to integer: %s", e.what());
                continue;
            }
            bounding_boxes.push_back(bbox);
        }

        // Process image
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        // Create a vector to store point clouds for each detected object
        std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> object_point_clouds;
        for (const auto& bbox : bounding_boxes) {
            object_point_clouds.push_back(pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>()));
        }

        for (size_t i = 0; i < cloud_camera_frame->points.size(); ++i) {
            const auto& point = cloud_camera_frame->points[i];
            if (point.z > 0) {
                cv::Point3d pt_cv(point.x, point.y, point.z);
                cv::Point2d uv = camera_model_.project3dToPixel(pt_cv);
                uv.y = image_height_ - uv.y; // Flip the y-coordinate to match the image coordinate system
                uv.x = image_width_ - uv.x;  // Mirror the x-coordinate

                // Check if the point is within any bounding box
                for (size_t bbox_idx = 0; bbox_idx < bounding_boxes.size(); ++bbox_idx) {
                    auto& bbox = bounding_boxes[bbox_idx];
                    if (bbox.valid && uv.x >= bbox.x_min && uv.x <= bbox.x_max && uv.y >= bbox.y_min && uv.y <= bbox.y_max) {
                        projected_points_.push_back(uv);  // Store valid points
                        bbox.sum_x += point.x;
                        bbox.sum_y += point.y;
                        bbox.sum_z += point.z;
                        bbox.count++;  // Increment count of points within this bbox

                        // Add point to object-specific point cloud
                        object_point_clouds[bbox_idx]->points.push_back(point);
                    }
                }
            }
        }

        // Publish object point clouds
        for (size_t bbox_idx = 0; bbox_idx < bounding_boxes.size(); ++bbox_idx) {
            const auto& bbox = bounding_boxes[bbox_idx];
            auto& object_cloud = object_point_clouds[bbox_idx];

            if (!object_cloud->points.empty()) {
                object_cloud->width = object_cloud->points.size();
                object_cloud->height = 1;
                object_cloud->is_dense = true;

                // Convert PCL point cloud to ROS message
                sensor_msgs::msg::PointCloud2 object_cloud_msg;
                pcl::toROSMsg(*object_cloud, object_cloud_msg);

                // Set the header
                object_cloud_msg.header.stamp = point_cloud_msg->header.stamp;
                object_cloud_msg.header.frame_id = camera_frame_;

                // Publish the point cloud
                object_point_cloud_publisher_->publish(object_cloud_msg);
            }
        }

        // Draw points on the image
        for (const auto& uv : projected_points_) {
            cv::circle(cv_ptr->image, cv::Point(uv.x, uv.y), 5, CV_RGB(255,0,0), -1);
        }

        // Publish modified image
        image_publisher_->publish(*cv_ptr->toImageMsg());

        // Publish detected object poses
        geometry_msgs::msg::PoseArray pose_array;
        pose_array.header.stamp = this->get_clock()->now();
        pose_array.header.frame_id = camera_frame_;

        for (const auto& bbox : bounding_boxes) {
            if (bbox.count > 0) {  // Ensure there is at least one point
                double avg_x = bbox.sum_x / bbox.count;
                double avg_y = bbox.sum_y / bbox.count;
                double avg_z = bbox.sum_z / bbox.count;

                // Create a new pose
                geometry_msgs::msg::Pose pose;
                pose.position.x = avg_x;
                pose.position.y = avg_y;
                pose.position.z = avg_z;
                pose.orientation.w = 1.0;  // Default orientation (no rotation)

                // Add the pose to the array
                pose_array.poses.push_back(pose);
            }
        }

        pose_publisher_->publish(pose_array);
    }

    // Member variables
    tf2_ros::Buffer tf_buffer_;  // TF2 buffer for transforms
    tf2_ros::TransformListener tf_listener_;  // TF2 transform listener
    image_geometry::PinholeCameraModel camera_model_;
    float min_depth_, max_depth_;
    std::string camera_frame_;
    unsigned int image_width_;
    unsigned int image_height_;
    std::vector<cv::Point2d> projected_points_;
    std::vector<BoundingBox> bounding_boxes;

    // Message filters subscribers
    message_filters::Subscriber<sensor_msgs::msg::PointCloud2> point_cloud_sub_;
    message_filters::Subscriber<sensor_msgs::msg::Image> image_sub_;
    message_filters::Subscriber<yolov8_msgs::msg::DetectionArray> detection_sub_;

    // Synchronizer
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2,
                                                           sensor_msgs::msg::Image,
                                                           yolov8_msgs::msg::DetectionArray> ApproximateSyncPolicy;
    typedef message_filters::Synchronizer<ApproximateSyncPolicy> Sync;
    std::shared_ptr<Sync> sync_;

    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_subscriber_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pose_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr object_point_cloud_publisher_;
};

int main(int argc, char **argv)
{
    // Initialize the ROS2 node
    rclcpp::init(argc, argv);
    // Create and spin the LidarCameraFusionNode
    auto node = std::make_shared<LidarCameraFusionNode>();
    rclcpp::spin(node);
    // Shutdown ROS2 cleanly
    rclcpp::shutdown();
    return 0;
}