#pragma once

#include <chrono>
#include <memory>

#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64.hpp"

namespace moveit_servo
{
class JogCalcs : public rclcpp::Node
{
public:
  JogCalcs(const rclcpp::NodeOptions & options);

private:
  void timer_callback();
  void jointStateCB(const std_msgs::msg::Float64::SharedPtr msg);
  void collisionScaleCB(const std_msgs::msg::Float64::SharedPtr msg);
  void twistCmdCB(const std_msgs::msg::Float64::SharedPtr msg);
  void jointCmdCB(const std_msgs::msg::Float64::SharedPtr msg);

  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr collision_scaling_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr twist_cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr joint_cmd_sub_;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr outgoing_cmd_pub_;
  
  mutable std::mutex latest_state_mutex_;
  double latest_joint_state_;
  double latest_collision_scale_;
  double latest_twist_cmd_;
  double latest_joint_cmd_;
  std::string status_;
};

}  // namespace moveit_servo