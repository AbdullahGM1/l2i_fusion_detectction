#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <yolov8_msgs/msg/detection_array.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <image_geometry/pinhole_camera_model.h>
#include <cv_bridge/cv_bridge.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/crop_box.h>
#include <pcl/common/transforms.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <opencv2/opencv.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// Define the main node class inheriting from rclcpp::Node
class LidarCameraFusionNode : public rclcpp::Node
{
public:
    // Constructor
    LidarCameraFusionNode()
        : Node("lidar_camera_fusion_node"),
          tf_buffer_(this->get_clock()),
          tf_listener_(tf_buffer_)
    {
        declare_parameters(); // Declare and retrieve parameters
        initialize_subscribers_and_publishers(); // Initialize ROS subscribers and publishers
    }

private:
    // Struct to hold bounding box information and associated points
    struct BoundingBox {
        double x_min, y_min, x_max, y_max; // Bounding box coordinates
        double sum_x = 0, sum_y = 0, sum_z = 0; // Sum of points for average calculation
        int count = 0; // Number of points within the bounding box
        bool valid = false; // Flag to indicate if the bounding box is valid
        int id = -1; // ID of the detection
        pcl::PointCloud<pcl::PointXYZ>::Ptr object_cloud = nullptr; // Point cloud of points within the bounding box
    };

    // Function to declare and retrieve parameters
    void declare_parameters()
    {
        declare_parameter<std::string>("lidar_frame", "x500_mono_1/lidar_link/gpu_lidar");
        declare_parameter<std::string>("camera_frame", "interceptor/gimbal_camera");
        declare_parameter<float>("min_range", 0.2);
        declare_parameter<float>("max_range", 10.0);

        get_parameter("lidar_frame", lidar_frame_);
        get_parameter("camera_frame", camera_frame_);
        get_parameter("min_range", min_range_);
        get_parameter("max_range", max_range_);

        // Log the parameters
        RCLCPP_INFO(
            get_logger(),
            "Parameters: lidar_frame='%s', camera_frame='%s', min_range=%.2f, max_range=%.2f",
            lidar_frame_.c_str(),
            camera_frame_.c_str(),
            min_range_,
            max_range_
        );
    }

    // Function to initialize ROS subscribers and publishers
    void initialize_subscribers_and_publishers()
    {
        // Subscribers for point cloud, image, and detection messages
        point_cloud_sub_.subscribe(this, "/scan/points");
        image_sub_.subscribe(this, "/interceptor/gimbal_camera");
        detection_sub_.subscribe(this, "/rgb/tracking");
        camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
            "/interceptor/gimbal_camera_info", 10, std::bind(&LidarCameraFusionNode::camera_info_callback, this, std::placeholders::_1));

        // Synchronizer to synchronize point cloud, image, and detection messages based on their timestamps
        using SyncPolicy = message_filters::sync_policies::ApproximateTime<
            sensor_msgs::msg::PointCloud2, sensor_msgs::msg::Image, yolov8_msgs::msg::DetectionArray>;
        sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(10), point_cloud_sub_, image_sub_, detection_sub_);
        sync_->registerCallback(std::bind(&LidarCameraFusionNode::sync_callback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

        // Publishers for the modified image, object poses, and object point clouds
        image_publisher_ = create_publisher<sensor_msgs::msg::Image>("/image_lidar_fusion", 10);
        pose_publisher_ = create_publisher<geometry_msgs::msg::PoseArray>("/detected_object_pose", 10);
        object_point_cloud_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>("/detected_object_point_cloud", 10);
    }

    // Callback function for camera info messages
    void camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
    {
        camera_model_.fromCameraInfo(msg); // Initialize camera model from camera info
        image_width_ = msg->width; // Store image width
        image_height_ = msg->height; // Store image height
    }

    // Synchronous callback function for synchronized messages
    void sync_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& point_cloud_msg,
                       const sensor_msgs::msg::Image::ConstSharedPtr& image_msg,
                       const yolov8_msgs::msg::DetectionArray::ConstSharedPtr& detection_msg)
    {
        // Process the point cloud and transform it to the camera frame
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_camera_frame = processPointCloud(point_cloud_msg);

        // Process the detections and create bounding boxes
        std::vector<BoundingBox> bounding_boxes = processDetections(detection_msg);

        // Project 3D points to 2D image coordinates and associate points with bounding boxes
        std::vector<cv::Point2d> projected_points = projectPointsAndAssociateWithBoundingBoxes(cloud_camera_frame, bounding_boxes);

        // Calculate the average poses of objects within bounding boxes
        geometry_msgs::msg::PoseArray pose_array = calculateObjectPoses(bounding_boxes, point_cloud_msg->header.stamp);

        // Prepare object point clouds for publishing
        std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> object_point_clouds;
        for (const auto& bbox : bounding_boxes) {
            if (bbox.count > 0 && bbox.object_cloud) {
                object_point_clouds.push_back(bbox.object_cloud);
            }
        }

        // Publish the modified image, object poses, and object point clouds
        publishResults(image_msg, projected_points, object_point_clouds, pose_array);
    }

    // Function to process and filter the point cloud, and transform it to the camera frame
    pcl::PointCloud<pcl::PointXYZ>::Ptr processPointCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& point_cloud_msg)
    {
        // Convert ROS point cloud message to PCL point cloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*point_cloud_msg, *cloud);

        // Apply a CropBox filter to remove points outside the defined range
        pcl::CropBox<pcl::PointXYZ> box_filter;
        box_filter.setInputCloud(cloud);
        box_filter.setMin(Eigen::Vector4f(min_range_, -max_range_, -max_range_, 1.0f));
        box_filter.setMax(Eigen::Vector4f(max_range_, max_range_, max_range_, 1.0f));
        box_filter.filter(*cloud);

        // Convert ROS time to rclcpp::Time for transform lookup
        rclcpp::Time cloud_time(point_cloud_msg->header.stamp);

        // Transform the point cloud from its original frame to the camera frame
        if (tf_buffer_.canTransform(camera_frame_, cloud->header.frame_id, cloud_time, tf2::durationFromSec(1.0))) {
            pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
            geometry_msgs::msg::TransformStamped transform = tf_buffer_.lookupTransform(camera_frame_, cloud->header.frame_id, cloud_time, tf2::durationFromSec(1.0));
            Eigen::Affine3d eigen_transform = tf2::transformToEigen(transform);
            pcl::transformPointCloud(*cloud, *transformed_cloud, eigen_transform);
            return transformed_cloud;
        }
        return cloud;
    }

    // Function to process detections and create bounding boxes
    std::vector<BoundingBox> processDetections(const yolov8_msgs::msg::DetectionArray::ConstSharedPtr& detection_msg)
    {
        std::vector<BoundingBox> bounding_boxes;
        for (const auto& detection : detection_msg->detections) {
            BoundingBox bbox;
            // Calculate bounding box coordinates from detection data
            bbox.x_min = detection.bbox.center.position.x - detection.bbox.size.x / 2.0;
            bbox.y_min = detection.bbox.center.position.y - detection.bbox.size.y / 2.0;
            bbox.x_max = detection.bbox.center.position.x + detection.bbox.size.x / 2.0;
            bbox.y_max = detection.bbox.center.position.y + detection.bbox.size.y / 2.0;
            bbox.valid = true; // Mark bounding box as valid

            // Convert detection ID to integer
            try {
                bbox.id = std::stoi(detection.id);
            } catch (const std::exception& e) {
                RCLCPP_ERROR(get_logger(), "Failed to convert detection ID to integer: %s", e.what());
                continue;
            }

            // Initialize point cloud for points within the bounding box
            bbox.object_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);

            // Add the bounding box to the list
            bounding_boxes.push_back(bbox);
        }
        return bounding_boxes;
    }

    // Function to project 3D points to 2D image coordinates and associate points with bounding boxes
    std::vector<cv::Point2d> projectPointsAndAssociateWithBoundingBoxes(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_camera_frame,
                                                                        std::vector<BoundingBox>& bounding_boxes)
    {
        std::vector<cv::Point2d> projected_points;
        for (const auto& point : cloud_camera_frame->points) {
            if (point.z > 0) { // Ensure point is in front of the camera
                cv::Point3d pt_cv(point.x, point.y, point.z); // Create a 3D point in CV coordinates
                cv::Point2d uv = camera_model_.project3dToPixel(pt_cv); // Project 3D point to 2D image coordinates

                // Adjust for image coordinate system (assuming origin at top-left)
                uv.y = image_height_ - uv.y;
                uv.x = image_width_ - uv.x;

                // Check if the projected point lies within any bounding box
                for (auto& bbox : bounding_boxes) {
                    if (bbox.valid && uv.x >= bbox.x_min && uv.x <= bbox.x_max && uv.y >= bbox.y_min && uv.y <= bbox.y_max) {
                        projected_points.push_back(uv); // Store the projected point
                        bbox.sum_x += point.x; // Accumulate point coordinates for average calculation
                        bbox.sum_y += point.y;
                        bbox.sum_z += point.z;
                        bbox.count++; // Increment point count
                        bbox.object_cloud->points.push_back(point); // Add point to the bounding box's point cloud
                    }
                }
            }
        }
        return projected_points;
    }

    // Function to calculate the average poses of objects within bounding boxes
    geometry_msgs::msg::PoseArray calculateObjectPoses(const std::vector<BoundingBox>& bounding_boxes,
                                                       const rclcpp::Time& cloud_time)
    {
        geometry_msgs::msg::PoseArray pose_array;
        pose_array.header.stamp = cloud_time;
        pose_array.header.frame_id = lidar_frame_; // Set pose array frame to lidar frame

        for (const auto& bbox : bounding_boxes) {
            if (bbox.count > 0) {
                // Calculate average position of points within the bounding box
                double avg_x = bbox.sum_x / bbox.count;
                double avg_y = bbox.sum_y / bbox.count;
                double avg_z = bbox.sum_z / bbox.count;

                // Create a pose in the camera frame
                geometry_msgs::msg::PoseStamped pose_camera;
                pose_camera.header.stamp = cloud_time;
                pose_camera.header.frame_id = camera_frame_;
                pose_camera.pose.position.x = avg_x;
                pose_camera.pose.position.y = avg_y;
                pose_camera.pose.position.z = avg_z;
                pose_camera.pose.orientation.w = 1.0; // Set to identity quaternion

                // Transform the pose from camera frame to lidar frame
                try {
                    geometry_msgs::msg::PoseStamped pose_lidar = tf_buffer_.transform(pose_camera, lidar_frame_, tf2::durationFromSec(1.0));
                    pose_array.poses.push_back(pose_lidar.pose); // Add the transformed pose to the pose array
                } catch (tf2::TransformException& ex) {
                    RCLCPP_ERROR(get_logger(), "Failed to transform pose: %s", ex.what());
                }
            }
        }
        return pose_array;
    }

    // Function to publish the modified image, object point clouds, and poses
    void publishResults(const sensor_msgs::msg::Image::ConstSharedPtr& image_msg,
                        const std::vector<cv::Point2d>& projected_points,
                        const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& object_point_clouds,
                        const geometry_msgs::msg::PoseArray& pose_array)
    {
        // Convert ROS image message to OpenCV image
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);

        // Draw circles at the projected points on the image
        for (const auto& uv : projected_points) {
            cv::circle(cv_ptr->image, cv::Point(uv.x, uv.y), 5, CV_RGB(255, 0, 0), -1);
        }

        // Publish the modified image
        image_publisher_->publish(*cv_ptr->toImageMsg());

        // Publish each object's point cloud
        for (const auto& object_cloud : object_point_clouds) {
            sensor_msgs::msg::PointCloud2 object_cloud_msg;
            pcl::toROSMsg(*object_cloud, object_cloud_msg);
            object_cloud_msg.header = image_msg->header;
            object_cloud_msg.header.frame_id = camera_frame_;
            object_point_cloud_publisher_->publish(object_cloud_msg);
        }

        // Publish the pose array
        pose_publisher_->publish(pose_array);
    }

    // TF buffer and listener for transform lookup
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    // Pinhole camera model for projecting 3D points to 2D image coordinates
    image_geometry::PinholeCameraModel camera_model_;
    // Parameters for point cloud filtering
    float min_range_, max_range_;
    std::string camera_frame_, lidar_frame_;
    int image_width_, image_height_;

    // Subscribers for point cloud, image, detection, and camera info messages
    message_filters::Subscriber<sensor_msgs::msg::PointCloud2> point_cloud_sub_;
    message_filters::Subscriber<sensor_msgs::msg::Image> image_sub_;
    message_filters::Subscriber<yolov8_msgs::msg::DetectionArray> detection_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;

    // Synchronizer for synchronizing point cloud, image, and detection messages
    std::shared_ptr<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2, sensor_msgs::msg::Image, yolov8_msgs::msg::DetectionArray>>> sync_;

    // Publishers for modified image, pose array, and object point clouds
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pose_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr object_point_cloud_publisher_;
};

// Main function to initialize ROS and spin the node
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LidarCameraFusionNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}