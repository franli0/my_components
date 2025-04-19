#include "my_components/attach_client_component.hpp"
#include <rclcpp_components/register_node_macro.hpp>

namespace my_components {

AttachClient::AttachClient(const rclcpp::NodeOptions & options)
: Node("attach_client", options),
  service_called_(false),
  service_completed_(false),
  waiting_for_response_(false),
  wait_count_(0),
  shutdown_counter_(0)
{
    RCLCPP_INFO(this->get_logger(), "Attach Client Component started.");
    
    // Create service client
    client_ = this->create_client<attach_shelf::srv::GoToLoading>("/approach_shelf");
    
    // Also subscribe to the lift topic to detect when the shelf is lifted
    lift_subscription_ = this->create_subscription<std_msgs::msg::String>(
        "/elevator_up", 10, 
        std::bind(&AttachClient::lift_callback, this, std::placeholders::_1));
    
    // Create timer to check service availability and make the call
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(500), 
        std::bind(&AttachClient::timer_callback, this));
    
    RCLCPP_INFO(this->get_logger(), "Waiting for /approach_shelf service...");
}

void AttachClient::lift_callback(const std_msgs::msg::String::SharedPtr /*msg*/)
{
    // When we detect the lift command, mark the operation as complete
    RCLCPP_INFO(this->get_logger(), "Detected shelf lifting command! Operation successful.");
    service_completed_ = true;
    RCLCPP_INFO(this->get_logger(), "Will shut down in 5 seconds...");
}

void AttachClient::timer_callback()
{
    // STATE 1: Initial state - waiting for service to be available and then calling it
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
        waiting_for_response_ = true;
        auto future = client_->async_send_request(
            request,
            [this](rclcpp::Client<attach_shelf::srv::GoToLoading>::SharedFuture future) {
                waiting_for_response_ = false;
                try {
                    auto result = future.get();
                    RCLCPP_INFO(this->get_logger(), "Service call acknowledged with result: %s", 
                              result->complete ? "success" : "failure");
                    
                    // Do NOT mark as completed here - wait for the lift callback
                    // The service completes earlier than the actual operations
                    
                } catch (const std::exception &e) {
                    RCLCPP_ERROR(this->get_logger(), "Service call failed: %s", e.what());
                    // If service fails, mark as completed to enable shutdown
                    service_completed_ = true;
                }
            });
            
        service_called_ = true;
        wait_count_ = 0;
        RCLCPP_INFO(this->get_logger(), "Service request sent, waiting for lift operation to complete...");
        return;
    }
    
    // Only proceed to shutdown if the lift operation was detected via the lift_callback
    if (service_completed_) {
        shutdown_counter_++;
        
        // Wait 5 seconds (10 ticks) before shutting down
        if (shutdown_counter_ >= 10) {
            RCLCPP_INFO(this->get_logger(), "Shutdown time reached, terminating now");
            rclcpp::shutdown();
        } else if (shutdown_counter_ % 2 == 0) {
            // Print countdown every second
            RCLCPP_INFO(this->get_logger(), "Shutting down in %d seconds...", 5 - (shutdown_counter_ / 2));
        }
        return;
    } else {
        // While waiting, print status periodically
        wait_count_++;
        if (wait_count_ % 10 == 0) {
            RCLCPP_INFO(this->get_logger(), "Waiting for shelf lifting operation to complete...");
        }
        
        // Safety timeout after 60 seconds (120 ticks at 500ms)
        if (wait_count_ > 120) {
            RCLCPP_ERROR(this->get_logger(), "Timeout waiting for shelf lifting operation. Forcing shutdown.");
            service_completed_ = true;
        }
    }
}

} // namespace my_components

RCLCPP_COMPONENTS_REGISTER_NODE(my_components::AttachClient)