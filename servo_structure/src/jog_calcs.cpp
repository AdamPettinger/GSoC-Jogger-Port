// MS2 Probably need a lisence here

#include "servo_structure/jog_calcs.h"

// For timing constants
using namespace std::chrono_literals;

// For binding the callback
using std::placeholders::_1;
using std::placeholders::_2;

namespace moveit_servo
{
JogCalcs::JogCalcs(const rclcpp::NodeOptions & options)
  : Node("jog_calcs", rclcpp::NodeOptions().use_intra_process_comms(true)), latest_joint_state_(0.0), 
  latest_collision_scale_(0.0), latest_twist_cmd_(0.0), latest_joint_cmd_(0.0), control_dim_(false), drift_dim_(false)
{
  status_pub_ = this->create_publisher<std_msgs::msg::String>("jog_calcs_status", 10);
  outgoing_cmd_pub_ = this->create_publisher<std_msgs::msg::Float64>("outgoing_cmds", 10);

  timer_ = this->create_wall_timer(
      3s, std::bind(&JogCalcs::timer_callback, this));

  collision_scaling_sub_ = this->create_subscription<std_msgs::msg::Float64>(
    "collision_velocity_scale", 10, std::bind(&JogCalcs::collisionScaleCB, this, _1));

  joint_state_sub_ = this->create_subscription<std_msgs::msg::Float64>(
    "joint_state", 10, std::bind(&JogCalcs::jointStateCB, this, _1));

  twist_cmd_sub_ = this->create_subscription<std_msgs::msg::Float64>(
    "twist_cmd", 10, std::bind(&JogCalcs::twistCmdCB, this, _1));

  joint_cmd_sub_ = this->create_subscription<std_msgs::msg::Float64>(
    "joint_jog_cmd", 10, std::bind(&JogCalcs::jointCmdCB, this, _1));

  control_dims_server_ = this->create_service<std_srvs::srv::SetBool>("control_dims", std::bind(&JogCalcs::changeControlDims, this, _1, _2));

  drift_dims_server_ = this->create_service<std_srvs::srv::SetBool>("drift_dims", std::bind(&JogCalcs::changeDriftDims, this, _1, _2));

  status_ = "No messages received yet";
}

void JogCalcs::timer_callback()
{
  double output, joint_state, collision_scale, twist_cmd, joint_cmd;
  std::string status;

  // Copy the latest joint state
  {
    const std::lock_guard<std::mutex> lock(latest_state_mutex_);
    joint_state = latest_joint_state_;
    collision_scale = latest_collision_scale_;
    twist_cmd = latest_twist_cmd_;
    joint_cmd = latest_joint_cmd_;

    status = status_;
  }

  output = joint_state + collision_scale + twist_cmd + joint_cmd;

  auto message = std_msgs::msg::Float64();
  message.data = output;

  std::cout << "[Calcs] Publishing output with value: " << message.data << " at address: " << &message << std::endl;
  outgoing_cmd_pub_->publish(message);

  auto status_msg = std_msgs::msg::String();
  status_msg.data = status;
  status_pub_->publish(status_msg);
}

void JogCalcs::jointStateCB(const std_msgs::msg::Float64::SharedPtr msg)
{
  std::cout << "[Calcs] Heard message with jointStateCB, value = " << msg->data << ". Address was " << msg.get() << std::endl;

  const std::lock_guard<std::mutex> lock(latest_state_mutex_);
  latest_joint_state_ = msg->data;
  status_ = "Last message recieved was: jointStateCB";
}

void JogCalcs::collisionScaleCB(const std_msgs::msg::Float64::SharedPtr msg)
{
  std::cout << "[Calcs] Heard message with collisionScaleCB, value = " << msg->data << ". Address was " << msg.get() << std::endl;

  const std::lock_guard<std::mutex> lock(latest_state_mutex_);
  latest_collision_scale_ = msg->data;
  status_ = "Last message recieved was: collisionScaleCB";
}

void JogCalcs::twistCmdCB(const std_msgs::msg::Float64::SharedPtr msg)
{
  std::cout << "[Calcs] Heard message with twistCmdCB, value = " << msg->data << ". Address was " << msg.get() << std::endl;

  const std::lock_guard<std::mutex> lock(latest_state_mutex_);
  latest_twist_cmd_ = msg->data;
  status_ = "Last message recieved was: twistCmdCB";
}

void JogCalcs::jointCmdCB(const std_msgs::msg::Float64::SharedPtr msg)
{
  std::cout << "[Calcs] Heard message with jointCmdCB, value = " << msg->data << ". Address was " << msg.get() << std::endl;

  const std::lock_guard<std::mutex> lock(latest_state_mutex_);
  latest_joint_cmd_ = msg->data;
  status_ = "Last message recieved was: jointCmdCB";
}

void JogCalcs::changeControlDims(const std::shared_ptr<std_srvs::srv::SetBool::Request> request, std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
  std::cout << "[Calcs] Got Control Request, value = " << request->data << ". Address was " << request.get() << std::endl;

  {
    const std::lock_guard<std::mutex> lock(latest_state_mutex_);
    control_dim_ = request->data;
    status_ = "Received ChangeControlDims service request";
  }

  response->success = true;
  response->message = "Setting control_dim_ to " + request->data ? "true" : "false";
}

void JogCalcs::changeDriftDims(const std::shared_ptr<std_srvs::srv::SetBool::Request> request, std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
  std::cout << "[Calcs] Got Drift Request, value = " << request->data << ". Address was " << request.get() << std::endl;

  {
    const std::lock_guard<std::mutex> lock(latest_state_mutex_);
    drift_dim_ = request->data;
    status_ = "Received changeDriftDims service request";
  }

  response->success = true;
  response->message = "Setting drift_dim_ to " + request->data ? "true" : "false";
}

}  // namespace moveit_servo

// Register the component with class_loader
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(moveit_servo::JogCalcs)