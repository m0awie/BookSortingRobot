#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <interfaces/srv/arm_command.hpp>  // Include custom service header

class BrainNode : public rclcpp::Node {
public:
    BrainNode() : Node("brain") {
        client_ = this->create_client<interfaces::srv::ArmCommand>("arm");

        // Wait for the service to be available
        while (!client_->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_WARN(this->get_logger(), "Waiting for 'arm' service to be available...");
        }
        
        sendTargetPose(0.1, 0.1, 0.1, 0.0, 0.0, 0.0);
    }

    void sendTargetPose(float x, float y, float z, float roll, float pitch, float yaw) {
        auto request = std::make_shared<interfaces::srv::ArmCommand::Request>();
        request->x = x;
        request->y = y;
        request->z = z;
        request->roll = roll;
        request->pitch = pitch;
        request->yaw = yaw;

        auto result_future = client_->async_send_request(request);

        // Handle response asynchronously
        auto future_result = result_future.wait_for(std::chrono::seconds(10));
        if (future_result == std::future_status::ready) {
            auto result = result_future.get();
            if (result->success) {
                RCLCPP_INFO(this->get_logger(), "Successfully moved to target pose.");
            } else {
                RCLCPP_WARN(this->get_logger(), "Failed to move to target pose.");
            }
        } else {
            RCLCPP_ERROR(this->get_logger(), "Service call timed out.");
        }
    }

private:
    rclcpp::Client<interfaces::srv::ArmCommand>::SharedPtr client_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BrainNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
