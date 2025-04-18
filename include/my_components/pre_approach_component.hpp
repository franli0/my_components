#ifndef PRE_APPROACH_COMPONENT_HPP
#define PRE_APPROACH_COMPONENT_HPP

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <cmath>

namespace my_components {

class PreApproach : public rclcpp::Node {
public:
    explicit PreApproach(const rclcpp::NodeOptions & options);

private:
    enum RobotState {
        MOVING_FORWARD,
        ROTATING,
        STOPPED
    };
    
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    double normalize_angle(double angle);
    double calculate_angular_difference();
    void timer_callback();
    
    // Node state variables
    RobotState state_;
    double obstacle_distance_; // Hardcoded value instead of parameter
    double rotation_degrees_;  // Hardcoded value instead of parameter
    double rotation_radians_;
    bool rotation_started_;
    int shutdown_counter_;
    bool standalone_mode_;     // True if running as standalone component
    
    // Orientation tracking
    double current_yaw_;
    double start_yaw_;
    double target_rotation_;
    double target_yaw_;
    
    // ROS interfaces
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
    rclcpp::TimerBase::SharedPtr timer_;
};

} // namespace my_components

#endif // PRE_APPROACH_COMPONENT_HPP