#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/static_transform_broadcaster.h"

class StaticTFPublisher : public rclcpp::Node
{
public:
  StaticTFPublisher()
  : Node("static_tf_map_to_odom")
  {
    static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

    geometry_msgs::msg::TransformStamped t;

    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "map";
    t.child_frame_id = "odom";

    t.transform.translation.x = 0.0;
    t.transform.translation.y = 0.0;
    t.transform.translation.z = 0.0;

    t.transform.rotation.x = 0.0;
    t.transform.rotation.y = 0.0;
    t.transform.rotation.z = 0.0;
    t.transform.rotation.w = 1.0;

    static_broadcaster_->sendTransform(t);
    RCLCPP_INFO(this->get_logger(), "Static TF map -> odom 퍼블리시 시작");
  }

private:
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<StaticTFPublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
