#include "rclcpp/rclcpp.hpp"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <opencv2/opencv.hpp>
#include <std_msgs/msg/float32.hpp>   
#include <geometry_msgs/msg/point.hpp>

class RealsenseBridge : public rclcpp::Node
{
public:
    RealsenseBridge() : Node("realsense_bridge")
    {
        depth_sub = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/camera_depth/depth/image_raw",
            rclcpp::QoS(rclcpp::KeepLast(20)).best_effort().durability_volatile(),
            std::bind(&RealsenseBridge::depth_callback,this,std::placeholders::_1)
        );
        depth_pub = this->create_publisher<std_msgs::msg::Float32>(
            "object_depth",   
            rclcpp::QoS(rclcpp::KeepLast(20)).best_effort().durability_volatile()       
        );
        pixel_xy_sub = this->create_subscription<geometry_msgs::msg::Point>(
            "pixel_xy",
            10,
            std::bind(&RealsenseBridge::pixel_callback,this,std::placeholders::_1)
        );

    }

private:
    void depth_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        cv_bridge::CvImagePtr cv_ptr;
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
        cv::Mat depth_img = cv_ptr->image;
        uint16_t depth_raw = depth_img.at<uint16_t>(cy, cx);
        if (depth_raw == 0) {
            RCLCPP_WARN(this->get_logger(), "Depth 값이 0 (유효하지 않음)");
            return;
        }

        float depth_m = static_cast<float>(depth_raw) * 0.001f;
        depth_publish(depth_m);
        RCLCPP_INFO(this->get_logger(), "Depth (m): %.3f", depth_m);
    }

    void pixel_callback(const geometry_msgs::msg::Point::SharedPtr msg){
        cx = msg->x;
        cy = msg->y;
        RCLCPP_INFO(this->get_logger(), "pixel _call back Depth (m): %.3f %.3f", cx,cy);
    }

    void depth_publish(const float depth_m){
        std_msgs::msg::Float32 depth_msg;
        depth_msg.data = depth_m;
        depth_pub->publish(depth_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr depth_pub;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr pixel_xy_sub;
    double cx = 0.0;
    double cy = 0.0;
    bool pixel_received;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RealsenseBridge>());
    rclcpp::shutdown();
    return 0;
}
