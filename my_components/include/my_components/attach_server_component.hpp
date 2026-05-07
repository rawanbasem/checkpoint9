#ifndef MY_COMPONENTS__ATTACH_SERVER_COMPONENT_HPP_
#define MY_COMPONENTS__ATTACH_SERVER_COMPONENT_HPP_

#include "my_components/visibility_control.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "std_msgs/msg/string.hpp"
#include "attach_shelf/srv/go_to_loading.hpp"
#include <tf2_ros/transform_broadcaster.h>
#include <vector>
#include <memory>

namespace my_components
{

class AttachServer : public rclcpp::Node
{
public:
  COMPOSITION_PUBLIC
  explicit AttachServer(const rclcpp::NodeOptions & options);

protected:
  void service_callback(
    const std::shared_ptr<attach_shelf::srv::GoToLoading::Request> request,
    std::shared_ptr<attach_shelf::srv::GoToLoading::Response> response);

  void laserscan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

  void broadcast_frame(float x, float y);
  void MoveToCoordinate(float rel_x, float rel_y);

private:
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::Service<attach_shelf::srv::GoToLoading>::SharedPtr service_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_subscriber_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr elevator_publisher_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  std::vector<std::pair<float, float>> detected_legs_;
  geometry_msgs::msg::Pose2D current_pos_;
};

}  // namespace my_components

#endif  // MY_COMPONENTS__ATTACH_SERVER_COMPONENT_HPP_