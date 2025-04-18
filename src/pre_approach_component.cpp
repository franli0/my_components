#include "my_components/pre_approach_component.hpp"
#include <rclcpp_components/register_node_macro.hpp>

namespace my_components {

PreApproach::PreApproach(const rclcpp::NodeOptions & options)
: Node("pre_approach", options),
  state_(MOVING_FORWARD),
  rotation_started_(false),
  shutdown_counter_(0),
  standalone_mode_(false),  // Default to false
  current_yaw_(0.0),
  start_yaw_(0.0),
  target_yaw_(0.0)
{
    // Hardcoded values (previously parameters)
    obstacle_distance_ = 0.5;  // 0.5 meters
    rotation_degrees_ = -90.0;  // -90 degrees
    
    // Convert degrees to radians
    rotation_radians_ = rotation_degrees_ * M_PI / 180.0;
    
    // Check for the standalone_mode parameter from the launch file
    this->declare_parameter<bool>("standalone_mode", true);  // Default to true for safety
    standalone_mode_ = this->get_parameter("standalone_mode").as_bool();
    
    RCLCPP_INFO(this->get_logger(), "Pre-Approach Component started in %s mode. Will stop at %f meters and rotate %f degrees",
                standalone_mode_ ? "standalone" : "integrated", obstacle_distance_, rotation_degrees_);
    
    // Create publisher for robot velocity
    vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
        "/diffbot_base_controller/cmd_vel_unstamped", 10);
    
    // Create subscription to laser scan
    scan_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10, std::bind(&PreApproach::scan_callback, this, std::placeholders::_1));
    
    // Subscribe to odometry for better rotation control
    odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10, std::bind(&PreApproach::odom_callback, this, std::placeholders::_1));
    
    // Timer for publishing velocity commands
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(50),  // Faster update rate for smoother control
        std::bind(&PreApproach::timer_callback, this));
        
    RCLCPP_INFO(this->get_logger(), "Publishing velocity commands to /diffbot_base_controller/cmd_vel_unstamped");
}

void PreApproach::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    if (state_ == MOVING_FORWARD)
    {
        // Get the range directly in front of the robot
        size_t front_index = msg->ranges.size() / 2;
        double front_distance = msg->ranges[front_index];
        
        // Check if we're close enough to the obstacle
        if (!std::isinf(front_distance) && front_distance < obstacle_distance_)
        {
            RCLCPP_INFO(this->get_logger(), "Obstacle detected at %f meters. Stopping and preparing to rotate.", 
                       front_distance);
            state_ = ROTATING;
            rotation_started_ = false;
        }
    }
}

void PreApproach::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    // Extract the yaw from the quaternion directly
    double x = msg->pose.pose.orientation.x;
    double y = msg->pose.pose.orientation.y;
    double z = msg->pose.pose.orientation.z;
    double w = msg->pose.pose.orientation.w;
    
    // Convert quaternion to yaw
    double siny_cosp = 2.0 * (w * z + x * y);
    double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
    current_yaw_ = std::atan2(siny_cosp, cosy_cosp);
}

double PreApproach::normalize_angle(double angle)
{
    // Normalize angle to be between -π and π
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

double PreApproach::calculate_angular_difference()
{
    // Calculate the angular difference (how much we've rotated so far)
    double difference = normalize_angle(current_yaw_ - start_yaw_);
    return difference;
}

void PreApproach::timer_callback()
{
    geometry_msgs::msg::Twist cmd_vel;
    
    switch (state_)
    {
        case MOVING_FORWARD:
        {
            // Move forward with perfect straight line (zero angular velocity)
            cmd_vel.linear.x = 0.2;
            cmd_vel.angular.z = 0.0;
            break;
        }
        case ROTATING:
        {
            if (!rotation_started_)
            {
                // Stop first
                cmd_vel.linear.x = 0.0;
                cmd_vel.angular.z = 0.0;
                vel_publisher_->publish(cmd_vel);
                
                // Record starting yaw angle
                start_yaw_ = current_yaw_;
                
                // Calculate target rotation amount (add small buffer to ensure full rotation)
                target_rotation_ = rotation_radians_ * 1.02; // Add 2% buffer
                
                RCLCPP_INFO(this->get_logger(), "Starting rotation from %f radians, target rotation: %f radians", 
                            start_yaw_, target_rotation_);
                
                rotation_started_ = true;
                return;
            }
            
            // Calculate how much we've rotated so far
            double rotated_amount = calculate_angular_difference();
            
            // Determine how much more we need to rotate (sign matters for direction)
            double remaining_rotation = target_rotation_ - rotated_amount;
            
            // Check if we've rotated enough
            if (std::abs(remaining_rotation) > 0.02) // ~1 degree threshold
            {
                // Continue rotating with proportional control for precision
                double k_p = 0.8; // Proportional gain
                double angular_velocity = std::max(-0.5, std::min(0.5, k_p * remaining_rotation));
                
                // Ensure minimum velocity to overcome static friction
                if (std::abs(angular_velocity) < 0.1 && std::abs(remaining_rotation) > 0.02) {
                    angular_velocity = (remaining_rotation > 0) ? 0.1 : -0.1;
                }
                
                cmd_vel.linear.x = 0.0;
                cmd_vel.angular.z = angular_velocity;
                
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                         "Rotating, rotated: %f, remaining: %f, cmd: %f", 
                         rotated_amount, remaining_rotation, angular_velocity);
            }
            else
            {
                // Stop rotating
                cmd_vel.linear.x = 0.0;
                cmd_vel.angular.z = 0.0;
                state_ = STOPPED;
                RCLCPP_INFO(this->get_logger(), "Rotation completed. Target: %f, Actual: %f, Error: %f degrees",
                            rotation_degrees_, rotated_amount * 180.0 / M_PI, 
                            (target_rotation_ - rotated_amount) * 180.0 / M_PI);
                
                // Start the shutdown countdown
                RCLCPP_INFO(this->get_logger(), "Task completed. Component will now %s...",
                           standalone_mode_ ? "shut down" : "remain active");
            }
            break;
        }
        case STOPPED:
        {
            // Do nothing, stay stopped
            cmd_vel.linear.x = 0.0;
            cmd_vel.angular.z = 0.0;
            
            // Increment shutdown counter
            shutdown_counter_++;
            
            // Only shutdown if we're in standalone mode
            if (shutdown_counter_ >= 60) {
                if (standalone_mode_) {
                    RCLCPP_INFO(this->get_logger(), "Task completed, shutting down...");
                    rclcpp::shutdown();
                } else {
                    RCLCPP_INFO(this->get_logger(), "Task completed, waiting for other components...");
                    // Reset counter to prevent repeated messages
                    shutdown_counter_ = 0;
                }
            }
            break;
        }
    }
    
    vel_publisher_->publish(cmd_vel);
}

} // namespace my_components

RCLCPP_COMPONENTS_REGISTER_NODE(my_components::PreApproach)