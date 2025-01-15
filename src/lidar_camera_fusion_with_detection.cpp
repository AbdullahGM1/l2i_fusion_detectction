#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <yolo_msgs/msg/detection_array.hpp>
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
          tf_buffer_(this->get_clock()),  // Initialize TF2 buffer
          tf_listener_(tf_buffer_)        // Initialize TF2 listener
    {
        declare_parameters();  // Declare and load parameters
        initialize_subscribers_and_publishers();  // Set up subscribers and publishers
    }

private:
    // Structure to hold bounding box information
    struct BoundingBox {
        double x_min, y_min, x_max, y_max;  // Bounding box coordinates in image space
        double sum_x = 0, sum_y = 0, sum_z = 0;  // Accumulated point coordinates for averaging
        int count = 0;  // Number of points in the bounding box
        bool valid = false;  // Flag to indicate if the bounding box is valid
        int id = -1;  // ID of the detected object
        pcl::PointCloud<pcl::PointXYZ>::Ptr object_cloud = nullptr;  // Point cloud for the object
    };

    // Declare and load parameters from the parameter server
    void declare_parameters()
    {
        declare_parameter<std::string>("lidar_frame", "x500_mono_1/lidar_link/gpu_lidar");
        declare_parameter<std::string>("camera_frame", "observer/gimbal_camera");
        declare_parameter<float>("min_range", 0.2);
        declare_parameter<float>("max_range", 10.0);

        get_parameter("lidar_frame", lidar_frame_);
        get_parameter("camera_frame", camera_frame_);
        get_parameter("min_range", min_range_);
        get_parameter("max_range", max_range_);

        RCLCPP_INFO(
            get_logger(),
            "Parameters: lidar_frame='%s', camera_frame='%s', min_range=%.2f, max_range=%.2f",
            lidar_frame_.c_str(),
            camera_frame_.c_str(),
            min_range_,
            max_range_
        );
    }

    // Initialize subscribers and publishers
    void initialize_subscribers_and_publishers()
    {
        // Subscribers for point cloud, image, and detections
        point_cloud_sub_.subscribe(this, "/scan/points");
        image_sub_.subscribe(this, "/observer/gimbal_camera");
        detection_sub_.subscribe(this, "/rgb/tracking");
        camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
            "/observer/gimbal_camera_info", 10, std::bind(&LidarCameraFusionNode::camera_info_callback, this, std::placeholders::_1));

        // Synchronizer to align point cloud, image, and detection messages
        using SyncPolicy = message_filters::sync_policies::ApproximateTime<
            sensor_msgs::msg::PointCloud2, sensor_msgs::msg::Image, yolo_msgs::msg::DetectionArray>;
        sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(10), point_cloud_sub_, image_sub_, detection_sub_);
        sync_->registerCallback(std::bind(&LidarCameraFusionNode::sync_callback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

        // Publishers for fused image, object poses, and object point clouds
        image_publisher_ = create_publisher<sensor_msgs::msg::Image>("/image_lidar_fusion", 10);
        pose_publisher_ = create_publisher<geometry_msgs::msg::PoseArray>("/detected_object_pose", 10);
        object_point_cloud_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>("/detected_object_point_cloud", 10);
    }

    // Callback for camera info to initialize the camera model
    void camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
    {
        camera_model_.fromCameraInfo(msg);  // Load camera intrinsics
        image_width_ = msg->width;  // Store image width
        image_height_ = msg->height;  // Store image height
    }

    // Synchronized callback for point cloud, image, and detections
    void sync_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& point_cloud_msg,
                       const sensor_msgs::msg::Image::ConstSharedPtr& image_msg,
                       const yolo_msgs::msg::DetectionArray::ConstSharedPtr& detection_msg)
    {
        // Process point cloud: crop, transform to camera frame
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_camera_frame = processPointCloud(point_cloud_msg);

        // Process detections: extract bounding boxes
        std::vector<BoundingBox> bounding_boxes = processDetections(detection_msg);

        // Project 3D points to 2D image space and associate with bounding boxes
        std::vector<cv::Point2d> projected_points = projectPointsAndAssociateWithBoundingBoxes(cloud_camera_frame, bounding_boxes);

        // Calculate object poses in the lidar frame
        geometry_msgs::msg::PoseArray pose_array = calculateObjectPoses(bounding_boxes, point_cloud_msg->header.stamp);

        // Publish results: fused image, object poses, and object point clouds
        publishResults(image_msg, projected_points, bounding_boxes, pose_array);
    }

    // Process point cloud: crop and transform to camera frame
    pcl::PointCloud<pcl::PointXYZ>::Ptr processPointCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& point_cloud_msg)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*point_cloud_msg, *cloud);  // Convert ROS message to PCL point cloud

        // Crop point cloud to a defined range
        pcl::CropBox<pcl::PointXYZ> box_filter;
        box_filter.setInputCloud(cloud);
        box_filter.setMin(Eigen::Vector4f(min_range_, -max_range_, -max_range_, 1.0f));
        box_filter.setMax(Eigen::Vector4f(max_range_, max_range_, max_range_, 1.0f));
        box_filter.filter(*cloud);

        // Transform point cloud to camera frame using TF2
        rclcpp::Time cloud_time(point_cloud_msg->header.stamp);
        if (tf_buffer_.canTransform(camera_frame_, cloud->header.frame_id, cloud_time, tf2::durationFromSec(1.0))) {
            geometry_msgs::msg::TransformStamped transform = tf_buffer_.lookupTransform(camera_frame_, cloud->header.frame_id, cloud_time, tf2::durationFromSec(1.0));
            Eigen::Affine3d eigen_transform = tf2::transformToEigen(transform); // Eigen::Affine3d - which is a 4x4 transformation matrix
            pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::transformPointCloud(*cloud, *transformed_cloud, eigen_transform);
            return transformed_cloud;
        }
        return cloud;  // Return original cloud if transformation fails
    }

    // Process detections: extract bounding boxes from YOLO detections
    std::vector<BoundingBox> processDetections(const yolo_msgs::msg::DetectionArray::ConstSharedPtr& detection_msg)
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
                bbox.id = std::stoi(detection.id);  // Convert detection ID to integer
            } catch (const std::exception& e) {
                RCLCPP_ERROR(get_logger(), "Failed to convert detection ID to integer: %s", e.what());
                continue;
            }
            bbox.object_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
            bounding_boxes.push_back(bbox);
        }
        return bounding_boxes;
    }

    // Project 3D points to 2D image space and associate with bounding boxes
    std::vector<cv::Point2d> projectPointsAndAssociateWithBoundingBoxes(
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_camera_frame,
        std::vector<BoundingBox>& bounding_boxes)
    {
        const int grid_size = 100;  // Size of the grid for spatial hashing
        const int grid_width = image_width_ / grid_size;
        const int grid_height = image_height_ / grid_size;
        std::vector<std::vector<BoundingBox*>> grid(grid_width * grid_height);  // Grid for spatial hashing

        // Populate the grid with bounding boxes
        for (auto& bbox : bounding_boxes) {
            if (!bbox.valid) continue;

            int x_min_grid = static_cast<int>(bbox.x_min / grid_size);
            int x_max_grid = static_cast<int>(bbox.x_max / grid_size);
            int y_min_grid = static_cast<int>(bbox.y_min / grid_size);
            int y_max_grid = static_cast<int>(bbox.y_max / grid_size);

            for (int x = x_min_grid; x <= x_max_grid; ++x) {
                for (int y = y_min_grid; y <= y_max_grid; ++y) {
                    if (x < 0 || x >= grid_width || y < 0 || y >= grid_height) continue;
                    grid[y * grid_width + x].push_back(&bbox);  // Add bounding box to grid cells
                }
            }
        }

        std::vector<cv::Point2d> projected_points;

        // Project each 3D point to 2D image space
        for (const auto& point : cloud_camera_frame->points) {
            if (point.z <= 0) continue;  // Skip points behind the camera

            cv::Point3d pt_cv(point.x, point.y, point.z);
            cv::Point2d uv = camera_model_.project3dToPixel(pt_cv);  // Project 3D point to 2D
            uv.y = image_height_ - uv.y;  // Adjust for image coordinate system
            uv.x = image_width_ - uv.x;

            // Find the grid cell for the projected point
            int grid_x = static_cast<int>(uv.x / grid_size);
            int grid_y = static_cast<int>(uv.y / grid_size);

            if (grid_x >= 0 && grid_x < grid_width && grid_y >= 0 && grid_y < grid_height) {
                // Check bounding boxes in the grid cell
                for (auto bbox_ptr : grid[grid_y * grid_width + grid_x]) {
                    if (uv.x >= bbox_ptr->x_min && uv.x <= bbox_ptr->x_max &&
                        uv.y >= bbox_ptr->y_min && uv.y <= bbox_ptr->y_max) {
                        projected_points.push_back(uv);  // Add projected point to results
                        bbox_ptr->sum_x += point.x;  // Accumulate point coordinates
                        bbox_ptr->sum_y += point.y;
                        bbox_ptr->sum_z += point.z;
                        bbox_ptr->count++;  // Increment point count
                        bbox_ptr->object_cloud->points.push_back(point);  // Add point to object cloud
                    }
                }
            }
        }

        return projected_points;
    }

    // Calculate object poses in the lidar frame
    geometry_msgs::msg::PoseArray calculateObjectPoses(
        const std::vector<BoundingBox>& bounding_boxes,
        const rclcpp::Time& cloud_time)
    {
        geometry_msgs::msg::PoseArray pose_array;
        pose_array.header.stamp = cloud_time;
        pose_array.header.frame_id = lidar_frame_;

        // Calculate average position for each bounding box and transform to lidar frame
        for (const auto& bbox : bounding_boxes) {
            if (bbox.count > 0) {
                double avg_x = bbox.sum_x / bbox.count;
                double avg_y = bbox.sum_y / bbox.count;
                double avg_z = bbox.sum_z / bbox.count;

                geometry_msgs::msg::PoseStamped pose_camera;
                pose_camera.header.stamp = cloud_time;
                pose_camera.header.frame_id = camera_frame_;
                pose_camera.pose.position.x = avg_x;
                pose_camera.pose.position.y = avg_y;
                pose_camera.pose.position.z = avg_z;
                pose_camera.pose.orientation.w = 1.0;

                try {
                    geometry_msgs::msg::PoseStamped pose_lidar = tf_buffer_.transform(pose_camera, lidar_frame_, tf2::durationFromSec(1.0));
                    pose_array.poses.push_back(pose_lidar.pose);  // Add transformed pose to results
                } catch (tf2::TransformException& ex) {
                    RCLCPP_ERROR(get_logger(), "Failed to transform pose: %s", ex.what());
                }
            }
        }
        return pose_array;
    }

    // Publish results: fused image, object poses, and object point clouds
    void publishResults(
        const sensor_msgs::msg::Image::ConstSharedPtr& image_msg,
        const std::vector<cv::Point2d>& projected_points,
        const std::vector<BoundingBox>& bounding_boxes,
        const geometry_msgs::msg::PoseArray& pose_array)
    {
        // Draw projected points on the image
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
        for (const auto& uv : projected_points) {
            cv::circle(cv_ptr->image, cv::Point(uv.x, uv.y), 5, CV_RGB(255, 0, 0), -1);
        }

        // Publish the fused image
        image_publisher_->publish(*cv_ptr->toImageMsg());

        // Publish object point clouds
        for (const auto& bbox : bounding_boxes) {
            if (bbox.count > 0 && bbox.object_cloud) {
                sensor_msgs::msg::PointCloud2 object_cloud_msg;
                pcl::toROSMsg(*bbox.object_cloud, object_cloud_msg);
                object_cloud_msg.header = image_msg->header;
                object_cloud_msg.header.frame_id = camera_frame_;
                object_point_cloud_publisher_->publish(object_cloud_msg);
            }
        }

        // Publish object poses
        pose_publisher_->publish(pose_array);
    }

    // TF2 buffer and listener for coordinate transformations
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    // Camera model for projecting 3D points to 2D image space
    image_geometry::PinholeCameraModel camera_model_;

    // Parameters for cropping and coordinate frames
    float min_range_, max_range_;
    std::string camera_frame_, lidar_frame_;
    int image_width_, image_height_;

    // Subscribers for point cloud, image, and detections
    message_filters::Subscriber<sensor_msgs::msg::PointCloud2> point_cloud_sub_;
    message_filters::Subscriber<sensor_msgs::msg::Image> image_sub_;
    message_filters::Subscriber<yolo_msgs::msg::DetectionArray> detection_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;

    // Synchronizer for aligning messages
    std::shared_ptr<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2, sensor_msgs::msg::Image, yolo_msgs::msg::DetectionArray>>> sync_;

    // Publishers for fused image, object poses, and object point clouds
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pose_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr object_point_cloud_publisher_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);  // Initialize ROS2
    auto node = std::make_shared<LidarCameraFusionNode>();  // Create node
    rclcpp::spin(node);  // Run node
    rclcpp::shutdown();  // Shutdown ROS2
    return 0;
}