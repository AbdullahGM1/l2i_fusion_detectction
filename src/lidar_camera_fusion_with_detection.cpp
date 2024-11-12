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
#include <yolov8_msgs/msg/detection_array.hpp>

class LidarCameraFusionNode : public rclcpp::Node
{
public:
    LidarCameraFusionNode() : Node("lidar_camera_fusion_node"),
                              tf_buffer_(std::make_shared<rclcpp::Clock>(), tf2::durationFromSec(10.0)),
                              tf_listener_(tf_buffer_)
    {
        this->declare_parameter<std::string>("config_file", "");
        this->get_parameter("config_file", config_file_);

        if (config_file_.empty()) {
            config_file_ = ament_index_cpp::get_package_share_directory("ros2_lidar_camera_fusion_with_detection_cpp") + "/config/setup_config.yaml";
        }

        if (!loadConfig()) {
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
        camera_model_ = std::make_unique<image_geometry::PinholeCameraModel>();
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Subscription<yolov8_msgs::msg::DetectionArray>::SharedPtr detection_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    std::unique_ptr<image_geometry::PinholeCameraModel> camera_model_;

    Eigen::Matrix4f transformation_matrix_;
    double min_depth_;
    double max_depth_;
    std::string config_file_;
    cv::Mat current_image_;
    std::vector<yolov8_msgs::msg::Detection> current_detections_;

    bool loadConfig()
    {
        try {
            YAML::Node config = YAML::LoadFile(config_file_);

            auto matrix = config["transformation_matrix"].as<std::vector<std::vector<double>>>();
            for (size_t i = 0; i < 4; ++i) {
                for (size_t j = 0; j < 4; ++j) {
                    transformation_matrix_(i, j) = static_cast<float>(matrix[i][j]);
                }
            }

            min_depth_ = config["depth_range"]["min"].as<double>();
            max_depth_ = config["depth_range"]["max"].as<double>();

            RCLCPP_INFO(this->get_logger(), "Configuration loaded successfully.");
            return true;
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error loading config file: %s", e.what());
            return false;
        }
    }

    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        if (!camera_model_->initialized()) {
            RCLCPP_WARN(this->get_logger(), "Camera model not initialized.");
            return;
        }

        auto cloud_filtered = filterPointCloud(msg);
        transformPointCloud(cloud_filtered, transformation_matrix_);
        projectPointCloudToImage(cloud_filtered);
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr filterPointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr& cloud_msg)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*cloud_msg, *cloud);
        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud(cloud);
        pass.setFilterFieldName("x");
        pass.setFilterLimits(min_depth_, max_depth_);
        pass.filter(*cloud);
        return cloud;
    }

    void transformPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const Eigen::Matrix4f& transform)
    {
        pcl::transformPointCloud(*cloud, *cloud, transform);
    }

    void projectPointCloudToImage(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
    {
        if (current_image_.empty()) {
            RCLCPP_WARN(this->get_logger(), "No image available for projection.");
            return;
        }

        for (const auto& point : cloud->points) {
            cv::Point3d pt_cv(point.x, point.y, point.z);
            if (pt_cv.x <= 0.0) continue;

            cv::Point2d uv = camera_model_->project3dToPixel(pt_cv);

            bool within_bbox = false;
            for (const auto& detection : current_detections_) {
                auto& bbox = detection.bbox;
                float x_min = bbox.center.position.x - bbox.size.x / 2;
                float y_min = bbox.center.position.y - bbox.size.y / 2;
                float x_max = bbox.center.position.x + bbox.size.x / 2;
                float y_max = bbox.center.position.y + bbox.size.y / 2;

                if (uv.x >= x_min && uv.x <= x_max && uv.y >= y_min && uv.y <= y_max) {
                    within_bbox = true;
                    break;
                }
            }

            if (within_bbox && uv.x >= 0 && uv.x < current_image_.cols && uv.y >= 0 && uv.y < current_image_.rows) {
                cv::circle(current_image_, uv, 5, CV_RGB(255, 0, 0), -1);
            }
        }

        auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", current_image_).toImageMsg();
        image_pub_->publish(*msg);
    }

    void detectionCallback(const yolov8_msgs::msg::DetectionArray::SharedPtr msg)
    {
        current_detections_ = msg->detections;
    }

    void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
    {
        camera_model_->fromCameraInfo(msg);
    }

    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        try {
            auto cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            current_image_ = cv_ptr->image.clone();
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        }
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LidarCameraFusionNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
