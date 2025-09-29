#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <nlohmann/json.hpp>
#include <deque>
#include <string>
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
using json = nlohmann::json;
struct Task {
  double x, y, z;
  double qx, qy, qz, qw;
};
class NavTaskNode : public rclcpp::Node {
public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleN2P = rclcpp_action::ClientGoalHandle<NavigateToPose>;
    NavTaskNode(const rclcpp::NodeOptions &options): Node("nav_task_node", options) {
        task_sub_ = this ->create_subscription<std_msgs::msg::String>(
            "/start_task",10,
            std::bind(&NavTaskNode::task_callback,this,std::placeholders::_1)
        );    

        nav_to_pose_client = rclcpp_action::create_client<NavigateToPose>(
            this,"navigate_to_pose"
        );

        amcl_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/amcl_pose", 10,
            std::bind(&NavTaskNode::amcl_callback, this, std::placeholders::_1)
        );
    }
private:
    void task_callback(const std_msgs::msg::String::SharedPtr msg){
        try {
            auto j = json::parse(msg->data);
            std::string robot = j["robot_name"]; //
            auto tasks = j["tasks"];
            for (auto &t : tasks) {
                Task task;
                task.x = t["pose"]["position"]["x"];
                task.y = t["pose"]["position"]["y"];
                task.z = t["pose"]["position"]["z"];
                task.qx = t["pose"]["orientation"]["x"];
                task.qy = t["pose"]["orientation"]["y"];
                task.qz = t["pose"]["orientation"]["z"];
                task.qw = t["pose"]["orientation"]["w"];
                
                task_queue_.push_back(task);
                RCLCPP_INFO(this->get_logger(), "Task position:");
                RCLCPP_INFO(this->get_logger(), "  x = %.3f", task.x);
                RCLCPP_INFO(this->get_logger(), "  y = %.3f", task.y);
                RCLCPP_INFO(this->get_logger(), "  z = %.3f", task.z);

                RCLCPP_INFO(this->get_logger(), "Task orientation:");
                RCLCPP_INFO(this->get_logger(), "  qx = %.3f", task.qx);
                RCLCPP_INFO(this->get_logger(), "  qy = %.3f", task.qy);
                RCLCPP_INFO(this->get_logger(), "  qz = %.3f", task.qz);
                RCLCPP_INFO(this->get_logger(), "  qw = %.3f", task.qw);
            }
            send_task_goal();
        } catch (std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "JSON parsing error: %s", e.what());
        }
    }

    void send_task_goal(){
        if (goal_in_progress_ || task_queue_.empty()) {
            return;  
        }
        Task next = task_queue_.front();
        task_queue_.pop_front();

        auto goal_msg = NavigateToPose::Goal();
        goal_msg.pose.header.frame_id = "map";
        goal_msg.pose.header.stamp = this->get_clock()->now();
        goal_msg.pose.pose.position.x = next.x;
        goal_msg.pose.pose.position.y = next.y;
        goal_msg.pose.pose.position.z = next.z;
        goal_msg.pose.pose.orientation.x = next.qx;
        goal_msg.pose.pose.orientation.y = next.qy;
        goal_msg.pose.pose.orientation.z = next.qz;
        goal_msg.pose.pose.orientation.w = next.qw;
        goal_in_progress_ = true;
        auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
        send_goal_options.result_callback =
            [this](const GoalHandleN2P::WrappedResult &result) {
                goal_in_progress_ = false;
                switch (result.code) {
                    case rclcpp_action::ResultCode::SUCCEEDED:
                        RCLCPP_INFO(this->get_logger(), "✅ Goal succeeded!");
                        break;
                    case rclcpp_action::ResultCode::ABORTED:
                        RCLCPP_WARN(this->get_logger(), "⚠️ Goal aborted by Nav2");
                        break;
                    case rclcpp_action::ResultCode::CANCELED:
                        RCLCPP_WARN(this->get_logger(), "⚠️ Goal canceled");
                        break;
                    default:
                        RCLCPP_ERROR(this->get_logger(), "Unknown result code");
                        break;
                }
                send_task_goal();
            };
        nav_to_pose_client->async_send_goal(goal_msg, send_goal_options);
    }

    void amcl_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
        current_amcl_pose_ = msg->pose.pose;
        if (!last_valid_pose_) {
            last_valid_pose_ = current_amcl_pose_;
            return;
        }
        double current_x = msg->pose.pose.position.x;
        double current_y = msg->pose.pose.position.y;
        double current_z = msg->pose.pose.position.z;
        double qx = msg->pose.pose.orientation.x;
        double qy = msg->pose.pose.orientation.y;
        double qz = msg->pose.pose.orientation.z;
        double qw = msg->pose.pose.orientation.w;
        RCLCPP_INFO(this->get_logger(),
            "📍 AMCL Pose:\n"
            "  Position -> x=%.3f, y=%.3f, z=%.3f\n"
            "  Orientation -> qx=%.3f, qy=%.3f, qz=%.3f, qw=%.3f",
            current_x, current_y, current_z,
            qx, qy, qz, qw
        );
        double dx = std::fabs(current_x - last_valid_pose_->position.x);
        double dy = std::fabs(current_y - last_valid_pose_->position.y);
        RCLCPP_INFO(this->get_logger(), "📍 dx 위치 변화 (%.2f m)", std::fabs(dx));
        RCLCPP_INFO(this->get_logger(), "📍 dx 위치 변화 (%.2f m)", std::fabs(dy));
        if(dx > 0.6 || dy > 0.6){
            if (goal_in_progress_ && nav_goal_handle_) {
                auto future_cancel = nav_to_pose_client->async_cancel_goal(nav_goal_handle_);
                goal_in_progress_ = false; 
            }
        }
        last_valid_pose_ = current_amcl_pose_;
    }

    std::deque<Task> task_queue_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr task_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_sub_;
    rclcpp_action::Client<NavigateToPose>::SharedPtr nav_to_pose_client;
    rclcpp_action::ClientGoalHandle<NavigateToPose>::SharedPtr nav_goal_handle_;
    bool goal_in_progress_{false};
    std::optional<geometry_msgs::msg::Pose> current_amcl_pose_;
    std::optional<geometry_msgs::msg::Pose> last_valid_pose_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NavTaskNode>(rclcpp::NodeOptions());
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
