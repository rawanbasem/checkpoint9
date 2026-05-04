#include "my_components/attach_client_component.hpp"
#include <memory>
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp" 
#include "attach_shelf/srv/go_to_loading.hpp"
#include <chrono>
#include <cmath>

using namespace std::chrono_literals;

namespace my_components {

AttachClient::AttachClient(const rclcpp::NodeOptions &options) 
: Node("attach_client_node", options) {
    
    publisher_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    laser_subscriber_ = create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", rclcpp::SensorDataQoS(), 
        std::bind(&AttachClient::laserscan_callback, this, std::placeholders::_1));

    subscription_ = create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10, std::bind(&AttachClient::odom_callback, this, std::placeholders::_1));

    client_ = create_client<attach_shelf::srv::GoToLoading>("/approach_shelf");

    timer_ = create_wall_timer(100ms, std::bind(&AttachClient::timer_callback, this));
    
    RCLCPP_INFO(this->get_logger(), "AttachClient component loaded and starting pre-approach...");
}

void AttachClient::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
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

void AttachClient::laserscan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) { 
    if (mode_ == "forward") {
        int middle_index = msg->ranges.size() / 2;
        float forward_dist = msg->ranges[middle_index];

        bool is_path_clear = !std::isfinite(forward_dist) || (forward_dist > obstacle_);

        if (!is_path_clear) {
            mode_ = "rotating";
            initial_angle_ = current_angle_; //  baseline for the relative turn
            RCLCPP_INFO(this->get_logger(), "Obstacle detected at: %.2f. Starting rotation from: %.2f deg", forward_dist, initial_angle_);
        }
    }
}

void AttachClient::timer_callback() {
    auto message = geometry_msgs::msg::Twist();

    if (mode_ == "forward") {
        message.linear.x = 0.5; 
        publisher_->publish(message);
    } 
    else if (mode_ == "rotating") {
        double turned_so_far = std::abs(current_angle_ - initial_angle_);
        if (turned_so_far < std::abs(degrees_)) {
            message.angular.z = (degrees_ > 0) ? 0.15 : -0.15;
            publisher_->publish(message);
        } else {
            mode_ = "finished";
        }
    } 
    else if (mode_ == "finished") {
        message.linear.x = 0.0;
        message.angular.z = 0.0;
        publisher_->publish(message);
        
        RCLCPP_INFO(this->get_logger(), "Pre-approach complete. Transitioning to service request.");
        mode_ = "requesting_service";
        this->handle_service_req();
    }
}

void AttachClient::handle_service_req() {
    if (!client_->wait_for_service(std::chrono::seconds(1))) {
      RCLCPP_ERROR(this->get_logger(), "Service /approach_shelf not available.");
      return;
    }

    auto request = std::make_shared<attach_shelf::srv::GoToLoading::Request>();
    request->attach_to_shelf = true; 
    
    RCLCPP_INFO(this->get_logger(), "Sending service request...");

    client_->async_send_request(request, 
      [this](rclcpp::Client<attach_shelf::srv::GoToLoading>::SharedFuture future) {
        auto response = future.get();
        if (response->complete) {
            RCLCPP_INFO(this->get_logger(), "Success! Robot is under shelf.");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failure during final approach.");
        }
        // rclcpp::shutdown(); 
    });
}

} // namespace my_components

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(my_components::AttachClient)