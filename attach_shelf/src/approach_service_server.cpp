#include "attach_shelf/srv/go_to_loading.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/string.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <rclcpp/rclcpp.hpp>
#include <algorithm>
#include <vector>
#include <cmath>

using namespace std::chrono_literals;

class ApproachService : public rclcpp::Node {
public:
  ApproachService() : Node("approach_service_node") {

    callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    rclcpp::SubscriptionOptions sub_options;
    sub_options.callback_group = callback_group_;

    service_ = this->create_service<attach_shelf::srv::GoToLoading>(
        "/approach_shelf", 
        std::bind(&ApproachService::service_callback, this, std::placeholders::_1, std::placeholders::_2),
        rmw_qos_profile_services_default,
        callback_group_);

    laser_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", rclcpp::SensorDataQoS(), 
        std::bind(&ApproachService::laserscan_callback, this, std::placeholders::_1),
        sub_options);

    odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10, 
        std::bind(&ApproachService::odom_callback, this, std::placeholders::_1),
        sub_options);

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    elevator_publisher_ = this->create_publisher<std_msgs::msg::String>("/elevator_up", 10);

    RCLCPP_INFO(this->get_logger(), "Approach Service Server Ready...");
  }

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

    void service_callback(
        const std::shared_ptr<attach_shelf::srv::GoToLoading::Request> request,
        std::shared_ptr<attach_shelf::srv::GoToLoading::Response> response) {

        if (detected_legs_.size() != 2) {
            RCLCPP_ERROR(this->get_logger(), "Detection failed: 1 leg found.");
            response->complete = false;
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Leg 1 found at: [x: %.3f, y: %.3f]", detected_legs_[0].first, detected_legs_[0].second);
        RCLCPP_INFO(this->get_logger(), "Leg 2 found at: [x: %.3f, y: %.3f]", detected_legs_[1].first, detected_legs_[1].second);

        float x_mid = (detected_legs_[0].first + detected_legs_[1].first) / 2.0;
        float y_mid = (detected_legs_[0].second + detected_legs_[1].second) / 2.0;
        
        RCLCPP_INFO(this->get_logger(), "Calculated Midpoint (Target): [x: %.3f, y: %.3f]", x_mid, y_mid);

        broadcast_frame(x_mid, y_mid);

        if (request->attach_to_shelf) {            
            MoveToCoordinate(x_mid, y_mid);
            response->complete = true;
        } else {
            response->complete = true;
        }
    }

    void laserscan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        bool is_detecting_leg = false;
        int start_idx = 0;
        std::vector<std::pair<float, float>> coordinates;

        for (size_t i = 0; i < msg->intensities.size(); i++) {
            // Detect start of high-intensity cluster
            if (!is_detecting_leg && msg->intensities[i] >= 8000.0) {
                is_detecting_leg = true;
                start_idx = static_cast<int>(i); 
            }
            // Detect end of cluster 
            else if (is_detecting_leg && msg->intensities[i] < 8000.0) {
                is_detecting_leg = false;
                
                int end_idx = static_cast<int>(i) - 1;
                int mid_idx = (start_idx + end_idx) / 2;
                
                float dist = msg->ranges[mid_idx];
                float angle = msg->angle_min + (static_cast<float>(mid_idx) * msg->angle_increment);
                
                coordinates.push_back({dist * std::cos(angle), dist * std::sin(angle)});
            }
        }

        // if leg cluster reaches the end of the scan
        if (is_detecting_leg) {
            int end_idx = static_cast<int>(msg->intensities.size()) - 1;
            int mid_idx = (start_idx + end_idx) / 2;
            
            float dist = msg->ranges[mid_idx];
            float angle = msg->angle_min + (static_cast<float>(mid_idx) * msg->angle_increment);
            
            coordinates.push_back({dist * std::cos(angle), dist * std::sin(angle)});
        }

        detected_legs_ = coordinates; 
    }

    void broadcast_frame(float x, float y) {

        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = this->get_clock()->now();
        t.header.frame_id = "robot_front_laser_base_link";
        t.child_frame_id = "cart_frame";   
        t.transform.translation.x = x; 
        t.transform.translation.y = y;
        t.transform.rotation.w = 1.0;
        tf_broadcaster_->sendTransform(t);
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {

        current_pos_.x = msg->pose.pose.position.x;
        current_pos_.y = msg->pose.pose.position.y;
        tf2::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
                          msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
        tf2::Matrix3x3 m(q);
        double r, p, y;
        m.getRPY(r, p, y);
        current_pos_.theta = y;
    }

    void MoveToCoordinate(float rel_x, float rel_y) {
        auto cmd_vel = geometry_msgs::msg::Twist();
        rclcpp::Rate loop_rate(10);

        // Transform laser-relative to global odom 
        double target_x = current_pos_.x + (rel_x * std::cos(current_pos_.theta) - rel_y * std::sin(current_pos_.theta));
        double target_y = current_pos_.y + (rel_x * std::sin(current_pos_.theta) + rel_y * std::cos(current_pos_.theta));

        // Reach midpoint
        double dist = 1.0;
        while (rclcpp::ok() && dist > 0.03) {
            double dx = target_x - current_pos_.x;
            double dy = target_y - current_pos_.y;
            dist = std::sqrt(dx*dx + dy*dy);
            double err_yaw = std::atan2(dy, dx) - current_pos_.theta;
            err_yaw = std::atan2(std::sin(err_yaw), std::cos(err_yaw));
            //RCLCPP_INFO(this->get_logger(), "Calculated error angle: [yaw: %f]", err_yaw);
            cmd_vel.linear.x = 0.3;
            cmd_vel.angular.z = 0.15*err_yaw;
            vel_publisher_->publish(cmd_vel);
            loop_rate.sleep();
        }

        // move 50 cm more, 30 cm is too lttle
        double start_x = current_pos_.x;
        double start_y = current_pos_.y;
        double moved = 0.0;
        while (rclcpp::ok() && moved < 0.5) { 
            moved = std::sqrt(std::pow(current_pos_.x - start_x, 2) + std::pow(current_pos_.y - start_y, 2));
            cmd_vel.linear.x = 0.15;
            cmd_vel.angular.z = 0.0;
            vel_publisher_->publish(cmd_vel);
            loop_rate.sleep();
        }

        cmd_vel.linear.x = 0.0;
        vel_publisher_->publish(cmd_vel);

        std_msgs::msg::String lift;
        lift.data = "up";
        elevator_publisher_->publish(lift);
        RCLCPP_INFO(this->get_logger(), "Shelf attached and lifted.");
    }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ApproachService>();
  
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  
  rclcpp::shutdown();
  return 0;
}