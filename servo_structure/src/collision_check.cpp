// MS2 Probably need a lisence here

#include "servo_structure/collision_check.h"

// For timing constants
using namespace std::chrono_literals;

// For binding the callback
using std::placeholders::_1;

namespace moveit_servo
{
CollisionCheck::CollisionCheck(const rclcpp::NodeOptions & options)
  : Node("collision_checking", rclcpp::NodeOptions().use_intra_process_comms(true)), latest_joint_state_(0.0)
{
  publisher_ = this->create_publisher<std_msgs::msg::Float64>("collision_velocity_scale", 10);
  timer_ = this->create_wall_timer(
      5s, std::bind(&CollisionCheck::timer_callback, this));

  subscription_ = this->create_subscription<std_msgs::msg::Float64>(
    "joint_state", 10, std::bind(&CollisionCheck::jointStateCB, this, _1));

  rclcpp::WaitSet wait_set(
    std::vector<rclcpp::WaitSet::SubscriptionEntry>{{subscription_}});

  {
    auto wait_result = wait_set.wait();
    std::cout << "Wait result was: " << wait_result.kind() << "\n";

    std_msgs::msg::Float64 msg;
    rclcpp::MessageInfo msg_info;
    subscription_->take(msg, msg_info);

    auto msg_ptr = std::make_shared<std_msgs::msg::Float64>(msg);
    jointStateCB(msg_ptr);

    std::cout << "End of waiting block\n";
  }
}

void CollisionCheck::timer_callback()
{
  double count;

  // Copy the latest joint state
  {
    const std::lock_guard<std::mutex> lock(CollisionCheck);
    count = latest_joint_state_;
  }

  auto message = std::make_unique<std_msgs::msg::Float64>();
  message.get()->data = 2*count;

  std::cout << "[Collision] Publishing output with value: " << message.get()->data << " at address: " << message.get() << std::endl;
  publisher_->publish(std::move(message));
}

void CollisionCheck::jointStateCB(const std_msgs::msg::Float64::SharedPtr msg)
{
  std::cout << "[Collision] Heard message with jointStateCB, value = " << msg->data << ". Address was " << msg.get() << std::endl;

  const std::lock_guard<std::mutex> lock(joint_state_mutex_);
  latest_joint_state_ = msg->data;
}

}  // namespace moveit_servo

// Register the component with class_loader
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(moveit_servo::CollisionCheck)