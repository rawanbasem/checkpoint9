#ifndef MY_COMPONENTS__ATTACH_CLIENT_COMPONENT_HPP_
#define MY_COMPONENTS__ATTACH_CLIENT_COMPONENT_HPP_

#include "my_components/visibility_control.h"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp" 
#include "attach_shelf/srv/go_to_loading.hpp"
#include <chrono>
#include <cmath>

namespace my_components {

class AttachClient : public rclcpp::Node {
public:
  COMPOSITION_PUBLIC
  explicit AttachClient(const rclcpp::NodeOptions &options);

protected:
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void laserscan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void timer_callback();
  void handle_service_req();

private:
  std::string mode_ = "forward";
  double obstacle_ = 0.4;
  double degrees_ = -90.0;
  double current_angle_;
  double initial_angle_;
  bool final_approach = true;
  
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_subscriber_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
  rclcpp::Client<attach_shelf::srv::GoToLoading>::SharedPtr client_;
  rclcpp::TimerBase::SharedPtr timer_;
};

} // namespace my_components

#endif // MY_COMPONENTS__ATTACH_CLIENT_COMPONENT_HPP_