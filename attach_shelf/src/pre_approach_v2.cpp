#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp" 
#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <cmath>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include "attach_shelf/srv/go_to_loading.hpp"
#include <memory>

using namespace std::chrono_literals;

class PreApproach : public rclcpp::Node {
public:
  PreApproach() : Node("pre_approach_node"), current_angle_(0.0), initial_angle_(0.0) {
    this->declare_parameter<double>("obstacle", 0.0);
    this->declare_parameter<double>("degrees", 0.0);
    this->declare_parameter<bool>("final_approach", false);
    
    obstacle_ = this->get_parameter("obstacle").as_double();
    degrees_ = this->get_parameter("degrees").as_double();
    final_approach = this->get_parameter("final_approach").as_bool();

    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    
    laser_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 
            rclcpp::SensorDataQoS(), 
            std::bind(&PreApproach::laserscan_callback, this, std::placeholders::_1));

    subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 
        10,
        std::bind(&PreApproach::odom_callback, this, std::placeholders::_1));

    client_ = this->create_client<attach_shelf::srv::GoToLoading>("/approach_shelf");

    timer_ = this->create_wall_timer(100ms, std::bind(&PreApproach::timer_callback, this));
    
    RCLCPP_INFO(this->get_logger(), "Node initialized. Moving toward wall...");
  }

private:
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
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

  void laserscan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) { 
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

  void timer_callback() {
    auto message = geometry_msgs::msg::Twist();
    double speed = 0.15;

    if (mode_ == "forward") {
        message.linear.x = 0.5; 
        message.angular.z = 0.0;
        publisher_->publish(message);
    } 
    else if (mode_ == "rotating") {
        double turned_so_far = std::abs(current_angle_ - initial_angle_);
        if (turned_so_far < std::abs(degrees_)) {
            message.linear.x = 0.0;
            message.angular.z = (degrees_ > 0) ? speed : -speed;
            publisher_->publish(message);
        } else {
            mode_ = "finished";
        }
    } 
    else if (mode_ == "finished") {
        message.linear.x = 0.0;
        message.angular.z = 0.0;
        publisher_->publish(message);
        
        RCLCPP_INFO(this->get_logger(), "Pre-approach rotation complete.");

        if (!final_approach) {
            RCLCPP_INFO(this->get_logger(), "final_approach is false. Shutting down...");
            rclcpp::shutdown();
        } else {
            // only executes ONCE
            mode_ = "requesting_service";
            this->handle_service_req();
        }
    }
  }

  void handle_service_req() {

    if (!client_->wait_for_service(std::chrono::seconds(1))) {
      RCLCPP_ERROR(this->get_logger(), "Service /approach_shelf not available. Shutting down.");
      rclcpp::shutdown();
      return;
    }

    auto request = std::make_shared<attach_shelf::srv::GoToLoading::Request>();
    request->attach_to_shelf = true; 
    
    RCLCPP_INFO(this->get_logger(), "Sending service request to /approach_shelf...");

    client_->async_send_request(request, 
      [this](rclcpp::Client<attach_shelf::srv::GoToLoading>::SharedFuture future) {
        auto response = future.get();
        if (response->complete) {
            RCLCPP_INFO(this->get_logger(), "Final approach successful! Robot is under shelf.");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Final approach failed (Perception detection error).");
        }
        rclcpp::shutdown();
    });
  }

  std::string mode_ = "forward";
  double obstacle_;
  double degrees_;
  double current_angle_;
  double initial_angle_;
  bool final_approach;
  
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_subscriber_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
  rclcpp::Client<attach_shelf::srv::GoToLoading>::SharedPtr client_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PreApproach>());
  rclcpp::shutdown();
  return 0;
}