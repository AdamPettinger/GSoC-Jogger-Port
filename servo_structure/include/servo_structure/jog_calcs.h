#pragma once

#include <chrono>
#include <memory>

#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_srvs/srv/set_bool.hpp"

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

  void changeControlDims(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
          std::shared_ptr<std_srvs::srv::SetBool::Response> response);

  void changeDriftDims(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
          std::shared_ptr<std_srvs::srv::SetBool::Response> response);

  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr collision_scaling_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr twist_cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr joint_cmd_sub_;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr outgoing_cmd_pub_;

  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr control_dims_server_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr drift_dims_server_;
  
  mutable std::mutex latest_state_mutex_;
  double latest_joint_state_;
  double latest_collision_scale_;
  double latest_twist_cmd_;
  double latest_joint_cmd_;
  std::string status_;
  bool control_dim_;
  bool drift_dim_;
};

}  // namespace moveit_servo