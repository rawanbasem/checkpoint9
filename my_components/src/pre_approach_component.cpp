#include "my_components/pre_approach_component.hpp"

#include <chrono>
#include <iostream>
#include <memory>
#include <utility>

#include "my_components/visibility_control.h"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp" 
#include <chrono>
#include <cmath>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

using namespace std::chrono_literals;

namespace my_components
{
PreApproach::PreApproach(const rclcpp::NodeOptions & options) : Node("pre_approach_node", options)
{
  laser_subscriber_ = create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 
            rclcpp::SensorDataQoS(), 
            std::bind(&PreApproach::laserscan_callback, this, std::placeholders::_1));
    
    subscription_ = create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 
            10,
            std::bind(&PreApproach::odom_callback, this, std::placeholders::_1));

  publisher_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

  timer_ = create_wall_timer(100ms, std::bind(&PreApproach::timer_callback, this));
}

void PreApproach::laserscan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) 
{ 
    if (mode_ == "forward") {
        int middle_index = msg->ranges.size() / 2;
        float forward_dist = msg->ranges[middle_index];

        bool is_path_clear = !std::isfinite(forward_dist) || (forward_dist > obstacle_);

        if (!is_path_clear) {
            mode_ = "rotating";
            initial_angle_ = current_angle_; // Set baseline for the relative turn
            RCLCPP_INFO(this->get_logger(), "Obstacle detected. Starting rotation from: %.2f deg", initial_angle_);
        }
    }
}

void PreApproach::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    tf2::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);

    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    // Convert radians to degrees
    current_angle_ = yaw * (180.0 / M_PI);
  }

void PreApproach::timer_callback()
{
    auto message = geometry_msgs::msg::Twist();

    if (mode_ == "forward") {
        message.linear.x = 0.5; 
        message.angular.z = 0.0;
    } 
    else if (mode_ == "rotating") {
        // Calculate change in angles
        double turned_so_far = std::abs(current_angle_ - initial_angle_);

        if (turned_so_far < std::abs(degrees_)) {
            message.linear.x = 0.0;
            message.angular.z = (degrees_ > 0) ? 0.2 : -0.2; // rad/s for rotation
        } else {
            mode_ = "finished";
        }
    } 
    else if (mode_ == "finished") {
        message.linear.x = 0.0;
        message.angular.z = 0.0;
        publisher_->publish(message);
        
        RCLCPP_INFO(this->get_logger(), "Rotation complete. Waiting for Final Approach...");
        timer_->cancel(); 
        return; 
    }

    publisher_->publish(message);
}

}

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(my_components::PreApproach)