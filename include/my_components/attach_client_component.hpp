#ifndef ATTACH_CLIENT_COMPONENT_HPP
#define ATTACH_CLIENT_COMPONENT_HPP

#include "rclcpp/rclcpp.hpp"
#include "attach_shelf/srv/go_to_loading.hpp"

namespace my_components {

class AttachClient : public rclcpp::Node {
public:
    explicit AttachClient(const rclcpp::NodeOptions & options);

private:
    // Service client
    rclcpp::Client<attach_shelf::srv::GoToLoading>::SharedPtr client_;
    
    // Timer to check service availability and call it
    rclcpp::TimerBase::SharedPtr timer_;
    
    // Flag to track if service has been called
    bool service_called_;
    
    // Shutdown counter
    int shutdown_counter_;
    
    // Timer callback
    void timer_callback();
};

} // namespace my_components

#endif // ATTACH_CLIENT_COMPONENT_HPP