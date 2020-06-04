#pragma once

#include <chrono>
#include <memory>

#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64.hpp"

namespace moveit_servo
{
class CollisionCheck : public rclcpp::Node
{
public:
  CollisionCheck(const rclcpp::NodeOptions & options);

private:
  void timer_callback();
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subscription_;
  void jointStateCB(const std_msgs::msg::Float64::SharedPtr msg);
  mutable std::mutex joint_state_mutex_;
  double latest_joint_state_;
};

}  // namespace moveit_servo