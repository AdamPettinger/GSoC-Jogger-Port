#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace moveit_servo
{
class CollisionCheck : public rclcpp::Node
{
public:
  CollisionCheck(const rclcpp::NodeOptions & options);

private:
  void timer_callback();
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  size_t count_;
};

}  // namespace moveit_servo