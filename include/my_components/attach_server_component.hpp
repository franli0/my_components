#ifndef ATTACH_SERVER_COMPONENT_HPP
#define ATTACH_SERVER_COMPONENT_HPP

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "attach_shelf/srv/go_to_loading.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <cmath>
#include <vector>
#include <memory>
#include <thread>
#include <chrono>
#include <algorithm>

using namespace std::chrono_literals;

namespace my_components {

class AttachServer : public rclcpp::Node {
public:
    explicit AttachServer(const rclcpp::NodeOptions & options);

private:
    enum RobotState {
        IDLE,
        DETECTING_LEGS,
        MOVING_TO_CART,
        MOVING_UNDER_SHELF,
        LIFTING_SHELF,
        COMPLETED,
        SHUTDOWN
    };
    
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void handle_service(
        const std::shared_ptr<attach_shelf::srv::GoToLoading::Request> request,
        std::shared_ptr<attach_shelf::srv::GoToLoading::Response> response);
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void detect_shelf_legs(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void calculate_cart_position();
    void publish_cart_transform();
    void timer_callback();
    void execute_move_to_cart();
    double normalize_angle(double angle);
    void execute_move_under_shelf();
    void execute_lift_shelf();
    void stop_robot();
    void complete_service(bool success);
    
    // State variables
    RobotState state_;
    bool service_in_progress_;
    bool attach_to_shelf_;
    bool legs_detected_;
    bool cart_frame_published_;
    int scan_count_;
    int shutdown_counter_;
    
    // Easily adjustable parameters
    double cart_position_offset_x_;
    double cart_position_offset_y_;
    double under_shelf_distance_;  // Distance to move under the shelf

    // Service request/response
    std::shared_ptr<attach_shelf::srv::GoToLoading::Request> current_request_;
    std::shared_ptr<attach_shelf::srv::GoToLoading::Response> pending_response_;
    
    // Position data
    double robot_x_;    // Robot x position in odom frame
    double robot_y_;    // Robot y position in odom frame
    double robot_yaw_;  // Robot orientation in odom frame
    double cart_x_;     // Cart position x in robot frame
    double cart_y_;     // Cart position y in robot frame
    double world_cart_x_; // Cart position x in odom frame
    double world_cart_y_; // Cart position y in odom frame
    double left_leg_x_;  // Left leg x position in robot frame
    double left_leg_y_;  // Left leg y position in robot frame  
    double right_leg_x_; // Right leg x position in robot frame
    double right_leg_y_; // Right leg y position in robot frame
    double under_shelf_target_x_; // Under-shelf position x in odom frame
    double under_shelf_target_y_; // Under-shelf position y in odom frame

    double movement_timer_ = 0.0;
    bool moving_forward_phase_ = false;
    bool turning_right_phase_ = false;
    
    // ROS interfaces
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_publisher_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr elevator_up_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
    rclcpp::Service<attach_shelf::srv::GoToLoading>::SharedPtr service_;
    std::unique_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;
};

} // namespace my_components

#endif // ATTACH_SERVER_COMPONENT_HPP