#ifndef ATTACH_CLIENT_COMPONENT_HPP
#define ATTACH_CLIENT_COMPONENT_HPP

#include "rclcpp/rclcpp.hpp"
#include "attach_shelf/srv/go_to_loading.hpp"
#include "std_msgs/msg/string.hpp"

namespace my_components {

class AttachClient : public rclcpp::Node {
public:
    explicit AttachClient(const rclcpp::NodeOptions & options);

private:
    // Service client
    rclcpp::Client<attach_shelf::srv::GoToLoading>::SharedPtr client_;
    
    // Subscription to detect lift commands
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr lift_subscription_;
    
    // Timer to check service availability and call it
    rclcpp::TimerBase::SharedPtr timer_;
    
    // Flag to track if service has been called
    bool service_called_;
    
    // Flag to track if service has completed
    bool service_completed_;
    
    // Flag to track if we're waiting for response
    bool waiting_for_response_;
    
    // Counter for waiting time
    int wait_count_;
    
    // Shutdown counter
    int shutdown_counter_;
    
    // Timer callback
    void timer_callback();
    
    // Lift topic callback
    void lift_callback(const std_msgs::msg::String::SharedPtr msg);
};

} // namespace my_components

#endif // ATTACH_CLIENT_COMPONENT_HPP