#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <image_geometry/pinhole_camera_model.h>
#include <cv_bridge/cv_bridge.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp> 
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <yolov8_msgs/msg/detection_array.hpp>
#include <geometry_msgs/msg/pose_array.hpp>


// Define the LidarCameraFusionNode class which inherits from rclcpp::Node
class LidarCameraFusionNode : public rclcpp::Node
{
public:
  // Constructor for the node
  LidarCameraFusionNode()
  : Node("lidar_camera_fusion_node"), // Initialize the node with a name
    tf_buffer_(std::make_shared<rclcpp::Clock>(), tf2::durationFromSec(10.0)), // Setup tf2 buffer with a 10 second storage
    tf_listener_(tf_buffer_), // Initialize tf2 listener to manage transformations
    lidar_frame_("x500_mono_1/lidar_link/gpu_lidar"),  // Predefined source frame for lidar data
    camera_frame_("interceptor/gimbal_camera")  // Predefined target frame for camera data
  {
    // Declare and retrieve parameters for filtering the point cloud
    this->declare_parameter<float>("min_depth", 0.2);
    this->declare_parameter<float>("max_depth", 10.0);
    this->get_parameter("min_depth", min_depth_);
    this->get_parameter("max_depth", max_depth_);

    // Create a subscription to the point cloud topic
    subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/scan/points", 10, std::bind(&LidarCameraFusionNode::point_cloud_callback, this, std::placeholders::_1));

    // Create a subscription to the camera info
    camera_info_subscriber_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        "/interceptor/gimbal_camera_info", 10, std::bind(&LidarCameraFusionNode::camera_info_callback, this, std::placeholders::_1));

    // Create a subscription to the camera image
    image_subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/interceptor/gimbal_camera", 10, std::bind(&LidarCameraFusionNode::image_callback, this, std::placeholders::_1));

    // Create a subscription to the YOLO BB
    detection_subscriber_ = this->create_subscription<yolov8_msgs::msg::DetectionArray>(
        "/yolo/tracking", 10, std::bind(&LidarCameraFusionNode::detection_callback, this, std::placeholders::_1));

    // Create a publisher for the lidar/camera fusion
    image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/image_lidar_fusion", 10);

    // Create a publisher for the detected object Pose
    pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/detected_object_pose", 10);

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
    // RCLCPP_INFO(this->get_logger(), "Camera model updated with width: %u and height: %u", image_width_, image_height_);

  }

  // Callback function to process YOLO tracking  
  void detection_callback(const yolov8_msgs::msg::DetectionArray::SharedPtr msg) {
    
    bounding_boxes.clear();  // Clear previous detections

    for (const auto& detection : msg->detections) {
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
  }


  // Callback function to process image  
  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    // Convert to OpenCV image
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    // Draw points
    for (const auto& uv : projected_points_) {
        cv::circle(cv_ptr->image, cv::Point(uv.x, uv.y), 5, CV_RGB(255,0,0), -1);
    }

    // Publish modified image
    image_publisher_->publish(*cv_ptr->toImageMsg());

  }


  // Callback function to process incoming point cloud data
  void point_cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    // Convert ROS point cloud message to PCL point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*msg, *cloud);

    // Apply the pass-through filter to limit points based on x values
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(min_depth_, max_depth_);
    pass.filter(*cloud_filtered);

    // Convert the filtered PCL point cloud back to ROS message format
    sensor_msgs::msg::PointCloud2 filtered_msg;
    pcl::toROSMsg(*cloud_filtered, filtered_msg);
    filtered_msg.header.stamp = msg->header.stamp;
    filtered_msg.header.frame_id = msg->header.frame_id;

    // Print the first 5 points from the filtered data for verification
    // RCLCPP_INFO(this->get_logger(), "First 5 points from filtered data:");
    // for (size_t i = 0; i < cloud_filtered->points.size() && i < 5; ++i)
    // {
    //     const auto& point = cloud_filtered->points[i];
    //     RCLCPP_INFO(this->get_logger(), "Filtered Point %zu: x=%f, y=%f, z=%f", i+1, point.x, point.y, point.z);
    // }

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

    // Print the first 5 points from the transformed data for verification
    // RCLCPP_INFO(this->get_logger(), "First 5 points after transformation:");
    // for (size_t i = 0; i < cloud_camera_frame->points.size() && i < 5; ++i)
    // {
    //     const auto& point = cloud_camera_frame->points[i];
    //     RCLCPP_INFO(this->get_logger(), "Transformed Point %zu: x=%f, y=%f, z=%f", i+1, point.x, point.y, point.z);
    // }

    projected_points_.clear();  // Clear previous points

    for (size_t i = 0; i < cloud_camera_frame->points.size(); ++i) {
        const auto& point = cloud_camera_frame->points[i];
        if (point.z > 0) {
            cv::Point3d pt_cv(point.x, point.y, point.z);
            cv::Point2d uv = camera_model_.project3dToPixel(pt_cv);
            uv.y = image_height_ - uv.y; // Flip the y-coordinate to match the image coordinate system
            uv.x = image_width_ - uv.x;  // Mirror the x-coordinate

             // Check if the point is within any bounding box
            for (auto& bbox : bounding_boxes) {
                if (bbox.valid && uv.x >= bbox.x_min && uv.x <= bbox.x_max && uv.y >= bbox.y_min && uv.y <= bbox.y_max) {
                    projected_points_.push_back(uv);  // Store valid points
                    bbox.sum_x += point.x;
                    bbox.sum_y += point.y;
                    bbox.sum_z += point.z;
                    bbox.count++;  // Increment count of points within this bbox
                }
            }
        }
    }

    geometry_msgs::msg::PoseArray pose_array;
    pose_array.header.stamp = this->get_clock()->now();
    pose_array.header.frame_id = camera_frame_; 

    // After processing all points, calculate the average for each bbox and print
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

            // RCLCPP_INFO(this->get_logger(), "BoundingBox ID %d Avg Pose: x=%f, y=%f, z=%f", bbox.id, avg_x, avg_y, avg_z);
        }
    }

    pose_publisher_->publish(pose_array);

}

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscriber_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_subscriber_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscriber_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
  rclcpp::Subscription<yolov8_msgs::msg::DetectionArray>::SharedPtr detection_subscriber_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pose_publisher_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  image_geometry::PinholeCameraModel camera_model_;
  float min_depth_, max_depth_;
  std::string lidar_frame_;
  std::string camera_frame_;
  unsigned int image_width_;
  unsigned int image_height_;
  cv::Mat current_image_;
  std::vector<cv::Point2d> projected_points_;
  std::vector<BoundingBox> bounding_boxes;
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
