#include "my_components/attach_server_component.hpp"
#include <chrono>
#include <cmath>
#include <memory>
#include <vector>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

using namespace std::chrono_literals;

namespace my_components
{

AttachServer::AttachServer(const rclcpp::NodeOptions & options)
: Node("attach_server_node", options)
{
    callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    rclcpp::SubscriptionOptions sub_options;
    sub_options.callback_group = callback_group_;

    service_ = this->create_service<attach_shelf::srv::GoToLoading>(
        "/approach_shelf", 
        std::bind(&AttachServer::service_callback, this, std::placeholders::_1, std::placeholders::_2),
        rmw_qos_profile_services_default,
        callback_group_);

    laser_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", rclcpp::SensorDataQoS(), 
        std::bind(&AttachServer::laserscan_callback, this, std::placeholders::_1),
        sub_options);

    odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10, 
        std::bind(&AttachServer::odom_callback, this, std::placeholders::_1),
        sub_options);

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    elevator_publisher_ = this->create_publisher<std_msgs::msg::String>("/elevator_up", 10);

    RCLCPP_INFO(this->get_logger(), "AttachServer Component Ready...");
}

void AttachServer::service_callback(
    const std::shared_ptr<attach_shelf::srv::GoToLoading::Request> request,
    std::shared_ptr<attach_shelf::srv::GoToLoading::Response> response)
{
    if (detected_legs_.size() != 2) {
        RCLCPP_ERROR(this->get_logger(), "Detection failed: %ld legs found.", detected_legs_.size());
        response->complete = false;
        return;
    }

    float x_mid = (detected_legs_[0].first + detected_legs_[1].first) / 2.0;
    float y_mid = (detected_legs_[0].second + detected_legs_[1].second) / 2.0;
    
    broadcast_frame(x_mid, y_mid);

    if (request->attach_to_shelf) {            
        MoveToCoordinate(x_mid, y_mid);
    }
    
    response->complete = true;
}

void AttachServer::laserscan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    bool is_detecting_leg = false;
    int start_idx = 0;
    std::vector<std::pair<float, float>> coordinates;

    for (size_t i = 0; i < msg->intensities.size(); i++) {
        if (!is_detecting_leg && msg->intensities[i] >= 8000.0) {
            is_detecting_leg = true;
            start_idx = static_cast<int>(i); 
        }
        else if (is_detecting_leg && msg->intensities[i] < 8000.0) {
            is_detecting_leg = false;
            int mid_idx = (start_idx + (static_cast<int>(i) - 1)) / 2;
            float dist = msg->ranges[mid_idx];
            float angle = msg->angle_min + (static_cast<float>(mid_idx) * msg->angle_increment);
            coordinates.push_back({dist * std::cos(angle), dist * std::sin(angle)});
        }
    }
    detected_legs_ = coordinates; 
}

void AttachServer::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    current_pos_.x = msg->pose.pose.position.x;
    current_pos_.y = msg->pose.pose.position.y;
    tf2::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
                      msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double r, p, yaw;
    m.getRPY(r, p, yaw);
    current_pos_.theta = yaw;
}

void AttachServer::broadcast_frame(float x, float y)
{
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "robot_front_laser_base_link";
    t.child_frame_id = "cart_frame";   
    t.transform.translation.x = x; 
    t.transform.translation.y = y;
    t.transform.rotation.w = 1.0;
    tf_broadcaster_->sendTransform(t);
}

void AttachServer::MoveToCoordinate(float rel_x, float rel_y)
{
    auto cmd_vel = geometry_msgs::msg::Twist();
    rclcpp::Rate loop_rate(10);

    // Calculate total distance to travel
    double dist_to_midpoint = std::sqrt(rel_x * rel_x + rel_y * rel_y);
    double total_distance = dist_to_midpoint + 0.5;

    double start_x = current_pos_.x;
    double start_y = current_pos_.y;
    double moved = 0.0;

    RCLCPP_INFO(this->get_logger(), "Starting straight final approach. Distance: %.3f m", total_distance);

    while (rclcpp::ok() && moved < total_distance) { 
        moved = std::sqrt(std::pow(current_pos_.x - start_x, 2) + std::pow(current_pos_.y - start_y, 2));
        
        cmd_vel.linear.x = 0.4;
        cmd_vel.angular.z = 0.0;
        
        vel_publisher_->publish(cmd_vel);
        loop_rate.sleep();
    }

    //  Stop
    cmd_vel.linear.x = 0.0;
    vel_publisher_->publish(cmd_vel);

    // Lift the shelf
    std_msgs::msg::String lift;
    lift.data = "up";
    elevator_publisher_->publish(lift);
    
    RCLCPP_INFO(this->get_logger(), "Shelf attached and lifted.");
}

} // namespace my_components

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(my_components::AttachServer)