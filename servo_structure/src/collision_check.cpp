// MS2 Probably need a lisence here

#include "servo_structure/collision_check.h"

// For timing constants
using namespace std::chrono_literals;

// For binding the callback
using std::placeholders::_1;

namespace moveit_servo
{
CollisionCheck::CollisionCheck(const rclcpp::NodeOptions & options)
  : Node("minimal_publisher", options), count_(0.0), latest_joint_state_(0.0)
{
  publisher_ = this->create_publisher<std_msgs::msg::Float64>("collision_velocity_scale", 10);
  timer_ = this->create_wall_timer(
      500ms, std::bind(&CollisionCheck::timer_callback, this));

  subscription_ = this->create_subscription<std_msgs::msg::Float64>(
    "joint_state", 10, std::bind(&CollisionCheck::jointStateCB, this, _1));
}

void CollisionCheck::timer_callback()
{
  // Copy the latest joint state
  {
    const std::lock_guard<std::mutex> lock(CollisionCheck);
    count_ = latest_joint_state_;
  }

  auto message = std_msgs::msg::Float64();
  message.data = 2*count_;

  std::cout << "Publishing output with value: " << message.data << " at address: " << &message << std::endl;
  publisher_->publish(message);
}

void CollisionCheck::jointStateCB(const std_msgs::msg::Float64::SharedPtr msg)
{
  std::cout << "Heard message with jointStateCB, value = " << msg->data << ". Address was " << &msg << std::endl;

  const std::lock_guard<std::mutex> lock(joint_state_mutex_);
  latest_joint_state_ = msg->data;
}

}  // namespace moveit_servo

// Register the component with class_loader
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(moveit_servo::CollisionCheck)