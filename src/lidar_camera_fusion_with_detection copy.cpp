#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/transforms.h>
#include <image_geometry/pinhole_camera_model.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>
#include <memory>
#include <geometry_msgs/msg/pose_array.hpp>
#include <yolov8_msgs/msg/detection_array.hpp>

class LidarCameraFusionNode : public rclcpp::Node
{
public:
    LidarCameraFusionNode() : Node("lidar_camera_fusion_node"),
                              tf_buffer_(std::make_shared<rclcpp::Clock>(), tf2::durationFromSec(10.0)),
                              tf_listener_(tf_buffer_),
                              camera_model_(std::make_unique<image_geometry::PinholeCameraModel>()),
                              image_width_(0), image_height_(0)
    {
        this->declare_parameter<std::string>("config_file", "");
        this->get_parameter("config_file", config_file_);

        if (config_file_.empty()) {
            config_file_ = ament_index_cpp::get_package_share_directory("ros2_lidar_camera_fusion_with_detection_cpp") + "/config/setup_config.yaml";
        }

        if (!loadConfig(config_file_)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load configuration file. Shutting down.");
            rclcpp::shutdown();
        }

        point_cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/scan/points", rclcpp::SensorDataQoS(), std::bind(&LidarCameraFusionNode::pointCloudCallback, this, std::placeholders::_1));
        camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            "/interceptor/gimbal_camera_info", rclcpp::SensorDataQoS(), std::bind(&LidarCameraFusionNode::cameraInfoCallback, this, std::placeholders::_1));
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/interceptor/gimbal_camera", rclcpp::SensorDataQoS(), std::bind(&LidarCameraFusionNode::imageCallback, this, std::placeholders::_1));
        detection_sub_ = this->create_subscription<yolov8_msgs::msg::DetectionArray>(
            "/yolo/tracking", rclcpp::QoS(10), std::bind(&LidarCameraFusionNode::detectionCallback, this, std::placeholders::_1));

        image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/image_lidar", 10);
        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/detected_object_position", 10);
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Subscription<yolov8_msgs::msg::DetectionArray>::SharedPtr detection_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pose_pub_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    std::unique_ptr<image_geometry::PinholeCameraModel> camera_model_;
    Eigen::Matrix4f transformation_matrix_;
    double min_depth_;
    double max_depth_;
    std::string config_file_;
    cv::Mat current_image_;
    std::vector<yolov8_msgs::msg::Detection> current_detections_;
    double fx_, fy_, cx_, cy_;
    int image_width_;
    int image_height_;

    bool loadConfig(const std::string &config_file)
    {
        try {
            YAML::Node config = YAML::LoadFile(config_file);
            auto matrix = config["transformation_matrix"].as<std::vector<std::vector<double>>>();

            std::cout << "Transformation Matrix:" << std::endl;
            for (size_t i = 0; i < 4; ++i) {
                for (size_t j = 0; j < 4; ++j) {
                    transformation_matrix_(i, j) = static_cast<float>(matrix[i][j]);
                    std::cout << transformation_matrix_(i, j) << " ";
                }
                std::cout << std::endl;
            }

            min_depth_ = config["depth_range"]["min"].as<double>();
            max_depth_ = config["depth_range"]["max"].as<double>();

            std::cout << "Minimum Depth: " << min_depth_ << std::endl;
            std::cout << "Maximum Depth: " << max_depth_ << std::endl;

            RCLCPP_INFO(this->get_logger(), "Configuration loaded successfully.");
            return true;
        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Error loading config file: %s", e.what());
            std::cout << "Error loading config file: " << e.what() << std::endl;
            return false;
        }
    }

    void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
    {
        fx_ = msg->k[0];
        fy_ = msg->k[4];
        cx_ = msg->k[2];
        cy_ = msg->k[5];
    }

    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        image_width_ = msg->width;
        image_height_ = msg->height;
        try {
            current_image_ = cv_bridge::toCvCopy(msg, "bgr8")->image;
        } catch (const cv_bridge::Exception &e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        }
    }

    void detectionCallback(const yolov8_msgs::msg::DetectionArray::SharedPtr msg)
    {
        current_detections_ = msg->detections;
    }

    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        if (current_image_.empty()) {
            RCLCPP_WARN(this->get_logger(), "No image received yet.");
            return;
        }

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *cloud);

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud(cloud);
        pass.setFilterFieldName("x");
        pass.setFilterLimits(min_depth_, max_depth_);
        pass.filter(*cloud_filtered);

        // Transform the point cloud using the transformation matrix
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_transformed(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::transformPointCloud(*cloud_filtered, *cloud_transformed, transformation_matrix_);

        cv::Mat image_with_points = current_image_.clone();
        std::map<int, std::vector<pcl::PointXYZ>> bbox_points;
        geometry_msgs::msg::PoseArray pose_array;
        pose_array.header.stamp = rclcpp::Clock().now();
        pose_array.header.frame_id = "lidar_frame";  // Adjust frame ID as necessary

        for (const auto &point : cloud_transformed->points) {
            double u = (fx_ * point.x / point.z) + cx_;
            double v = (fy_ * point.y / point.z) + cy_;

            for (int i = 0; i < current_detections_.size(); i++) {
                auto &detection = current_detections_[i];
                auto &bbox = detection.bbox;
                double x_min = bbox.center.position.x - bbox.size.x / 2.0;
                double y_min = bbox.center.position.y - bbox.size.y / 2.0;
                double x_max = bbox.center.position.x + bbox.size.x / 2.0;
                double y_max = bbox.center.position.y + bbox.size.y / 2.0;

                if (u >= x_min && u <= x_max && v >= y_min && v <= y_max) {
                    bbox_points[i].push_back(point);
                    cv::circle(image_with_points, cv::Point(u, v), 3, cv::Scalar(0, 255, 0), -1);
                    break;
                }
            }
        }

        publishDetectedObjectsPose(bbox_points, pose_array);
        publishImageWithPoints(image_with_points);
    }

    void publishDetectedObjectsPose(const std::map<int, std::vector<pcl::PointXYZ>>& bbox_points, geometry_msgs::msg::PoseArray& pose_array)
    {
        for (auto &[id, points] : bbox_points) {
            if (!points.empty()) {
                double sum_x = 0, sum_y = 0, sum_z = 0;
                for (auto &p : points) {
                    sum_x += p.x;
                    sum_y += p.y;
                    sum_z += p.z;
                }
                double avg_x = sum_x / points.size();
                double avg_y = sum_y / points.size();
                double avg_z = sum_z / points.size();

                geometry_msgs::msg::Pose pose;
                pose.position.x = avg_x;
                pose.position.y = avg_y;
                pose.position.z = avg_z;
                pose.orientation.w = 1.0; // No rotation information
                pose_array.poses.push_back(pose);

                // RCLCPP_INFO(this->get_logger(), "BB%d Average pose: X = %f, Y = %f, Z = %f", id+1, avg_x, avg_y, avg_z);
            }
        }

        // Publish the pose array
        pose_pub_->publish(pose_array);
    }

    void publishImageWithPoints(const cv::Mat& image_with_points)
    {
        auto output_image = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", image_with_points).toImageMsg();
        image_pub_->publish(*output_image);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LidarCameraFusionNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}