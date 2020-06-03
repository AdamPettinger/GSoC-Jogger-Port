#include "subscriber_member_function.cpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>(rclcpp::NodeOptions()));
  rclcpp::shutdown();
  return 0;
}