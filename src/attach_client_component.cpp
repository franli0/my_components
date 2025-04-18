#include "my_components/attach_client_component.hpp"
#include <rclcpp_components/register_node_macro.hpp>

namespace my_components {

AttachClient::AttachClient(const rclcpp::NodeOptions & options)
: Node("attach_client", options),
  service_called_(false),
  shutdown_counter_(0)
{
    RCLCPP_INFO(this->get_logger(), "Attach Client Component started.");
    
    // Create service client
    client_ = this->create_client<attach_shelf::srv::GoToLoading>("/approach_shelf");
    
    // Create timer to check service availability and make the call
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(500), 
        std::bind(&AttachClient::timer_callback, this));
    
    RCLCPP_INFO(this->get_logger(), "Waiting for /approach_shelf service...");
}

void AttachClient::timer_callback()
{
    if (!service_called_) {
        // Check if service is available
        if (!client_->wait_for_service(std::chrono::milliseconds(100))) {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
                "Waiting for /approach_shelf service to be available...");
            return;
        }
        
        RCLCPP_INFO(this->get_logger(), "Service /approach_shelf is available, sending request...");
        
        // Create request
        auto request = std::make_shared<attach_shelf::srv::GoToLoading::Request>();
        request->attach_to_shelf = true;  // Always set to true for full approach
        
        // Send async request
        auto future = client_->async_send_request(
            request,
            [this](rclcpp::Client<attach_shelf::srv::GoToLoading>::SharedFuture future) {
                try {
                    auto result = future.get();
                    RCLCPP_INFO(this->get_logger(), "Service call completed with result: %s", 
                              result->complete ? "success" : "failure");
                } catch (const std::exception &e) {
                    RCLCPP_ERROR(this->get_logger(), "Service call failed: %s", e.what());
                }
                
                // Start shutdown countdown
                RCLCPP_INFO(this->get_logger(), "Client operation complete, will shutdown in 3 seconds...");
            });
            
        service_called_ = true;
    } else {
        // Just increment shutdown counter once service has been called
        shutdown_counter_++;
        
        // When done, shutdown completely (this component is always runtime-loaded)
        if (shutdown_counter_ >= 6) {
            RCLCPP_INFO(this->get_logger(), "Client operation completed, shutting down...");
            rclcpp::shutdown();  // This will shut down the container and all components
        }
    }
}

} // namespace my_components

RCLCPP_COMPONENTS_REGISTER_NODE(my_components::AttachClient)