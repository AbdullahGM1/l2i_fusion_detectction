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

class LidarCameraFusionNode : public rclcpp::Node
{
public:
    LidarCameraFusionNode()
        : Node("lidar_camera_fusion_node"),
          tf_buffer_(this->get_clock()),
          tf_listener_(tf_buffer_)
    {
        declare_parameters();
        initialize_subscribers_and_publishers();
    }

private:
    struct BoundingBox {
        double x_min, y_min, x_max, y_max;
        double sum_x = 0, sum_y = 0, sum_z = 0;
        int count = 0;
        bool valid = false;
        int id = -1;
        pcl::PointCloud<pcl::PointXYZ>::Ptr object_cloud = nullptr;
    };

    void declare_parameters()
    {
        declare_parameter<std::string>("lidar_frame", "x500_mono_1/lidar_link/gpu_lidar");
        declare_parameter<std::string>("camera_frame", "interceptor/gimbal_camera");
        declare_parameter<float>("min_depth", 0.2);
        declare_parameter<float>("max_depth", 10.0);

        get_parameter("lidar_frame", lidar_frame_);
        get_parameter("camera_frame", camera_frame_);
        get_parameter("min_depth", min_depth_);
        get_parameter("max_depth", max_depth_);

         // Print all parameters in one line
        RCLCPP_INFO(
            get_logger(),
            "Parameters: lidar_frame='%s', camera_frame='%s', min_depth=%.2f, max_depth=%.2f",
            lidar_frame_.c_str(),
            camera_frame_.c_str(),
            min_depth_,
            max_depth_
        );             
    }

    void initialize_subscribers_and_publishers()
    {
        // Subscribers
        point_cloud_sub_.subscribe(this, "/scan/points");
        image_sub_.subscribe(this, "/interceptor/gimbal_camera");
        detection_sub_.subscribe(this, "/rgb/tracking");
        camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
            "/interceptor/gimbal_camera_info", 10, std::bind(&LidarCameraFusionNode::camera_info_callback, this, std::placeholders::_1));

        // Synchronizer
        using SyncPolicy = message_filters::sync_policies::ApproximateTime<
            sensor_msgs::msg::PointCloud2, sensor_msgs::msg::Image, yolov8_msgs::msg::DetectionArray>;
        sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(10), point_cloud_sub_, image_sub_, detection_sub_);
        sync_->registerCallback(std::bind(&LidarCameraFusionNode::sync_callback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

        // Publishers
        image_publisher_ = create_publisher<sensor_msgs::msg::Image>("/image_lidar_fusion", 10);
        pose_publisher_ = create_publisher<geometry_msgs::msg::PoseArray>("/detected_object_pose", 10);
        object_point_cloud_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>("/detected_object_point_cloud", 10);
    }

    void camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
    {
        camera_model_.fromCameraInfo(msg);
        image_width_ = msg->width;
        image_height_ = msg->height;
    }

    void sync_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& point_cloud_msg,
                       const sensor_msgs::msg::Image::ConstSharedPtr& image_msg,
                       const yolov8_msgs::msg::DetectionArray::ConstSharedPtr& detection_msg)
    {
        // Process point cloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_camera_frame = processPointCloud(point_cloud_msg);

        // Process detections
        std::vector<BoundingBox> bounding_boxes = processDetections(detection_msg);

        // Project points and associate with bounding boxes
        std::vector<cv::Point2d> projected_points = projectPointsAndAssociateWithBoundingBoxes(cloud_camera_frame, bounding_boxes);

        // Calculate object poses
        geometry_msgs::msg::PoseArray pose_array = calculateObjectPoses(bounding_boxes, point_cloud_msg->header.stamp);

        // Prepare object point clouds for publishing
        std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> object_point_clouds;
        for (const auto& bbox : bounding_boxes) {
            if (bbox.count > 0 && bbox.object_cloud) {
                object_point_clouds.push_back(bbox.object_cloud);
            }
        }

        // Publish results
        publishResults(image_msg, projected_points, object_point_clouds, pose_array);
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr processPointCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& point_cloud_msg)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*point_cloud_msg, *cloud);

        // Apply CropBox filter
        pcl::CropBox<pcl::PointXYZ> box_filter;
        box_filter.setInputCloud(cloud);
        box_filter.setMin(Eigen::Vector4f(min_depth_, -max_depth_, -max_depth_, 1.0f));
        box_filter.setMax(Eigen::Vector4f(max_depth_, max_depth_, max_depth_, 1.0f));
        box_filter.filter(*cloud);

        // Convert the timestamp to rclcpp::Time
        rclcpp::Time cloud_time(point_cloud_msg->header.stamp);

        // Transform point cloud into camera frame
        if (tf_buffer_.canTransform(camera_frame_, cloud->header.frame_id, cloud_time, tf2::durationFromSec(1.0))) {
            pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
            geometry_msgs::msg::TransformStamped transform = tf_buffer_.lookupTransform(camera_frame_, cloud->header.frame_id, cloud_time, tf2::durationFromSec(1.0));
            Eigen::Affine3d eigen_transform = tf2::transformToEigen(transform);
            pcl::transformPointCloud(*cloud, *transformed_cloud, eigen_transform);
            return transformed_cloud;
        }
        return cloud;
    }

    std::vector<BoundingBox> processDetections(const yolov8_msgs::msg::DetectionArray::ConstSharedPtr& detection_msg)
    {
        std::vector<BoundingBox> bounding_boxes;
        for (const auto& detection : detection_msg->detections) {
            BoundingBox bbox;
            bbox.x_min = detection.bbox.center.position.x - detection.bbox.size.x / 2.0;
            bbox.y_min = detection.bbox.center.position.y - detection.bbox.size.y / 2.0;
            bbox.x_max = detection.bbox.center.position.x + detection.bbox.size.x / 2.0;
            bbox.y_max = detection.bbox.center.position.y + detection.bbox.size.y / 2.0;
            bbox.valid = true;
            try {
                bbox.id = std::stoi(detection.id);
            } catch (const std::exception& e) {
                RCLCPP_ERROR(get_logger(), "Failed to convert detection ID to integer: %s", e.what());
                continue;
            }
            bbox.object_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
            bounding_boxes.push_back(bbox);
        }
        return bounding_boxes;
    }

    std::vector<cv::Point2d> projectPointsAndAssociateWithBoundingBoxes(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_camera_frame,
                                                                        std::vector<BoundingBox>& bounding_boxes)
    {
        std::vector<cv::Point2d> projected_points;
        for (const auto& point : cloud_camera_frame->points) {
            if (point.z > 0) {
                cv::Point3d pt_cv(point.x, point.y, point.z);
                cv::Point2d uv = camera_model_.project3dToPixel(pt_cv);
                uv.y = image_height_ - uv.y; // Adjust for image coordinate system
                uv.x = image_width_ - uv.x;

                for (auto& bbox : bounding_boxes) {
                    if (bbox.valid && uv.x >= bbox.x_min && uv.x <= bbox.x_max && uv.y >= bbox.y_min && uv.y <= bbox.y_max) {
                        projected_points.push_back(uv);
                        bbox.sum_x += point.x;
                        bbox.sum_y += point.y;
                        bbox.sum_z += point.z;
                        bbox.count++;
                        bbox.object_cloud->points.push_back(point);
                    }
                }
            }
        }
        return projected_points;
    }

    geometry_msgs::msg::PoseArray calculateObjectPoses(const std::vector<BoundingBox>& bounding_boxes,
                                                       const rclcpp::Time& cloud_time)
    {
        geometry_msgs::msg::PoseArray pose_array;
        pose_array.header.stamp = cloud_time;
        pose_array.header.frame_id = lidar_frame_; // Set to lidar frame

        for (const auto& bbox : bounding_boxes) {
            if (bbox.count > 0) {
                double avg_x = bbox.sum_x / bbox.count;
                double avg_y = bbox.sum_y / bbox.count;
                double avg_z = bbox.sum_z / bbox.count;

                // Create a Pose in camera frame
                geometry_msgs::msg::PoseStamped pose_camera;
                pose_camera.header.stamp = cloud_time;
                pose_camera.header.frame_id = camera_frame_;
                pose_camera.pose.position.x = avg_x;
                pose_camera.pose.position.y = avg_y;
                pose_camera.pose.position.z = avg_z;
                pose_camera.pose.orientation.w = 1.0;

                // Transform pose to lidar frame
                try {
                    geometry_msgs::msg::PoseStamped pose_lidar = tf_buffer_.transform(pose_camera, lidar_frame_, tf2::durationFromSec(1.0));
                    pose_array.poses.push_back(pose_lidar.pose);
                } catch (tf2::TransformException& ex) {
                    RCLCPP_ERROR(get_logger(), "Failed to transform pose: %s", ex.what());
                }
            }
        }
        return pose_array;
    }

    void publishResults(const sensor_msgs::msg::Image::ConstSharedPtr& image_msg,
                        const std::vector<cv::Point2d>& projected_points,
                        const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& object_point_clouds,
                        const geometry_msgs::msg::PoseArray& pose_array)
    {
        // Draw points on the image
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
        for (const auto& uv : projected_points) {
            cv::circle(cv_ptr->image, cv::Point(uv.x, uv.y), 5, CV_RGB(255, 0, 0), -1);
        }

        // Publish modified image
        image_publisher_->publish(*cv_ptr->toImageMsg());

        // Publish object point clouds
        for (const auto& object_cloud : object_point_clouds) {
            sensor_msgs::msg::PointCloud2 object_cloud_msg;
            pcl::toROSMsg(*object_cloud, object_cloud_msg);
            object_cloud_msg.header = image_msg->header;
            object_cloud_msg.header.frame_id = camera_frame_;
            object_point_cloud_publisher_->publish(object_cloud_msg);
        }

        // Publish poses
        pose_publisher_->publish(pose_array);
    }

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    image_geometry::PinholeCameraModel camera_model_;
    float min_depth_, max_depth_;
    std::string camera_frame_, lidar_frame_;
    int image_width_, image_height_;

    // Subscribers
    message_filters::Subscriber<sensor_msgs::msg::PointCloud2> point_cloud_sub_;
    message_filters::Subscriber<sensor_msgs::msg::Image> image_sub_;
    message_filters::Subscriber<yolov8_msgs::msg::DetectionArray> detection_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;

    // Synchronizer
    std::shared_ptr<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2, sensor_msgs::msg::Image, yolov8_msgs::msg::DetectionArray>>> sync_;

    // Publishers
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pose_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr object_point_cloud_publisher_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LidarCameraFusionNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}