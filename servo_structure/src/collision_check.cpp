// MS2 Probably need a lisence here

#include "servo_structure/collision_check.h"

// For timing constants
using namespace std::chrono_literals;

namespace moveit_servo
{
CollisionCheck::CollisionCheck(const rclcpp::NodeOptions & options)
  : Node("minimal_publisher", options), count_(0)
{
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    timer_ = this->create_wall_timer(
        500ms, std::bind(&CollisionCheck::timer_callback, this));
}

void CollisionCheck::timer_callback()
{
    auto message = std_msgs::msg::String();
    message.data = "Hello, world! " + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s' at address %p", message.data.c_str(),
        &message);
    publisher_->publish(message);
}

}  // namespace moveit_servo

// Register the component with class_loader
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(moveit_servo::CollisionCheck)