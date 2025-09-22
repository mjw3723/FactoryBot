#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

class CmdVelBridge : public rclcpp::Node
{
public:
    CmdVelBridge() : Node("cmd_vel_bridge")
    {
        // 구독: TwistStamped 타입
        sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
            "/cmd_vel", 10,
            std::bind(&CmdVelBridge::callback, this, std::placeholders::_1));

        // 퍼블리시: Twist 타입
        pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        RCLCPP_INFO(this->get_logger(), "CmdVelBridge node started (TwistStamped -> Twist)");
    }

private:
    void callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
    {
        geometry_msgs::msg::Twist twist_msg;
        twist_msg.linear = msg->twist.linear;
        twist_msg.angular = msg->twist.angular;
        twist_msg.linear.x *= 0.5;
        twist_msg.linear.y *= 0.5;
        twist_msg.linear.z *= 0.5;

        twist_msg.angular.x *= 0.5;
        twist_msg.angular.y *= 0.5;
        twist_msg.angular.z *= 0.5;
        pub_->publish(twist_msg);
    }

    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CmdVelBridge>());
    rclcpp::shutdown();
    return 0;
}
