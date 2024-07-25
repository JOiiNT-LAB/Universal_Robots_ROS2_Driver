#include <memory>
#include <string>
#include <chrono>
#include <cstdlib>
#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "controller_manager_msgs/msg/controller_state.hpp"
#include "controller_manager_msgs/srv/list_controllers.hpp"
#include "controller_manager_msgs/srv/switch_controller.hpp"
#include "ur_dashboard_msgs/msg/robot_mode.hpp"
#include "ur_dashboard_msgs/msg/robot_freedrive_state.hpp"

#include "std_msgs/msg/bool.hpp" 
#include "std_srvs/srv/empty.hpp"

using namespace std::chrono_literals;

int state(0);
int old_state(2);
bool new_state_(true);
controller_manager_msgs::srv::ListControllers::Response controller_list_response;

void enable_freedrive_callback(const std::shared_ptr<std_srvs::srv::Empty::Request> request, std::shared_ptr<std_srvs::srv::Empty::Response> response) {
  new_state_ = true;
  
  if(old_state == 1){
    state = 2; // enable freedrive
  }else if(old_state == 2){
    state = 1; // disable freedrive
  }
  old_state = state;
}

// Funzione per stampare il contenuto di switch_request
void print_switch_request(const std::shared_ptr<controller_manager_msgs::srv::SwitchController::Request> &request)
{
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "SwitchController Request:");

    // Funzione per stampare il contenuto di un vettore di stringhe
    auto print_string_vector = [](const std::vector<std::string>& vec) {
        if (vec.empty()) {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "None");
        } else {
            std::string output = "[ ";
            for (const auto& str : vec) {
                output += str + " ";
            }
            output += "]";
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "%s", output.c_str());
        }
    };

    // Stampa i campi del messaggio
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Activate Controllers:");
    print_string_vector(request->activate_controllers);

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Deactivate Controllers:");
    print_string_vector(request->deactivate_controllers);
}

int main(int argc, char **argv)
{
  // ROS2 Node initialization
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("freedrive_manager");

  // ROS2 Publisher
  auto robot_state_pub = node->create_publisher<ur_dashboard_msgs::msg::RobotFreedriveState>("~/freedrive_state", 10);
  auto freedrive_state_msg = ur_dashboard_msgs::msg::RobotFreedriveState();

  // ROS2 Service Servers
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr service =
    node->create_service<std_srvs::srv::Empty>("~/enable_freedrive", &enable_freedrive_callback);

  // ROS2 Service Clients
  auto list_controller_client_ = node->create_client<controller_manager_msgs::srv::ListControllers>("controller_manager/list_controllers");
  auto switch_controller_client_ = node->create_client<controller_manager_msgs::srv::SwitchController>("controller_manager/switch_controller");
  
  auto hw_freedrive_client = node->create_client<std_srvs::srv::Empty>("hardware_interface/enable_freedrive");

  // ROS2 service Requests
  auto list_controller_request  = std::make_shared<controller_manager_msgs::srv::ListControllers::Request>();
  auto switch_request           = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
  auto freedrive_request        = std::make_shared<std_srvs::srv::Empty::Request>();

  while (rclcpp::ok()) {
    switch (state) {
      case 0:
      {
        if (new_state_) {
          RCLCPP_INFO(node->get_logger(), "Idle state: waiting request to enable freedrive mode");
          new_state_ = false;
          controller_list_response.controller.clear();
        }
        break;
      }
      case 1:
      {
        if (new_state_) {
          RCLCPP_INFO(node->get_logger(), "ENABLE Request received: disabling control and launching freedrive mode");
          new_state_ = false;
        }

        // Wait for list_controllers service to become available
        while (!list_controller_client_->wait_for_service(1s)) {
          if (!rclcpp::ok()) {
            RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for the service. Exiting.");
            return 1;
          }
          RCLCPP_INFO(node->get_logger(), "Service not available, waiting again...");
        }

        // Send request and handle response
        auto controller_list = list_controller_client_->async_send_request(list_controller_request);
        if (rclcpp::spin_until_future_complete(node, controller_list) == rclcpp::FutureReturnCode::SUCCESS) {
          controller_list_response = *controller_list.get();

          std::cout << "Listing controllers:\n";
          // Add all the controllers apart from joint_state_broadcaster to the switch controller request
          for (const auto& controller : controller_list_response.controller) {
            if (controller.name == "joint_state_broadcaster") {
              RCLCPP_WARN(node->get_logger(), "Skipping deactivation of joint_state_broadcaster");
              continue;
            }

            // If the controller is active, prepare to stop it
            if (controller.state == "active") {
              switch_request->deactivate_controllers.push_back(controller.name);  // Use deactivate_controllers
              switch_request->strictness = controller_manager_msgs::srv::SwitchController::Request::BEST_EFFORT;
            }
          }

          // Send switch_controller request to deactivate everything apart joint_state_broadcaster
          if (switch_request->deactivate_controllers.size() != 0) {
            auto switch_response_future = switch_controller_client_->async_send_request(switch_request);
            if (rclcpp::spin_until_future_complete(node, switch_response_future) == rclcpp::FutureReturnCode::SUCCESS) {
              auto switch_response = switch_response_future.get();
              if (switch_response->ok) {
                // RCLCPP_INFO(node->get_logger(), "Successfully switched controllers.");
              } else {
                RCLCPP_ERROR(node->get_logger(), "Failed to switch controllers.");
              }
            } else {
              RCLCPP_ERROR(node->get_logger(), "Failed to call service switch_controller");
              return 1;
            }
          } else {
            RCLCPP_WARN(node->get_logger(), "switch_request has no controller to deactivate!");
          }

          // Wait for hw_freedrive_client service to become available
          while (!hw_freedrive_client->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
              RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for the service. Exiting.");
              return 1;
            }
            RCLCPP_INFO(node->get_logger(), "Service not available, waiting again...");
          }

          // Send request to enable freedrive mode
          auto hw_freedrive_response = hw_freedrive_client->async_send_request(freedrive_request);
          if (rclcpp::spin_until_future_complete(node, hw_freedrive_response) == rclcpp::FutureReturnCode::SUCCESS) {
            auto hw_freedrive_response_data = hw_freedrive_response.get();
            // RCLCPP_INFO(node->get_logger(), "Successfully enabled freedrive mode");
          } else {
            RCLCPP_ERROR(node->get_logger(), "Failed to enable freedrive mode");
            return 1;
          }

        } else {
          RCLCPP_ERROR(node->get_logger(), "Failed to call service list_controllers");
          return 1;
        }
        state = 0;
        break;
      }
      // Disable freedrive mode
      case 2:
      {
        if (new_state_) {
          RCLCPP_INFO(node->get_logger(), "DISABLE Request received: disabling freedrive mode, enabling back the controllers");
          new_state_ = false;
        }

        // Wait for disable freedrive service to become available
        while (!hw_freedrive_client->wait_for_service(1s)) {
          if (!rclcpp::ok()) {
            RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for the service. Exiting.");
            return 1;
          }
          RCLCPP_INFO(node->get_logger(), "Service not available, waiting again...");
        }
        // Sending disable freedrive request
        auto hw_freedrive_response = hw_freedrive_client->async_send_request(freedrive_request);
        if (rclcpp::spin_until_future_complete(node, hw_freedrive_response) == rclcpp::FutureReturnCode::SUCCESS) {
          auto hw_freedrive_response_data = hw_freedrive_response.get();
          RCLCPP_INFO(node->get_logger(), "Successfully disabled freedrive mode");
        } else {
          RCLCPP_ERROR(node->get_logger(), "Failed to disable freedrive mode");
          return 1;
        }

        std::cout << "------------------------------------------------------------" << std::endl;
        auto temp_switch_request = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
        temp_switch_request->strictness = controller_manager_msgs::srv::SwitchController::Request::BEST_EFFORT;

        for (const auto& controller : switch_request->deactivate_controllers) {
          temp_switch_request->activate_controllers.push_back(controller);
          print_switch_request(temp_switch_request);

          auto temp_switch_response_future = switch_controller_client_->async_send_request(temp_switch_request);
          if (rclcpp::spin_until_future_complete(node, temp_switch_response_future) == rclcpp::FutureReturnCode::SUCCESS) {
            auto switch_response = temp_switch_response_future.get();
            if (switch_response->ok) {
              temp_switch_request->activate_controllers.clear();
              print_switch_request(temp_switch_request);
              std::cout << "------------------------------------------------------------" << std::endl;
              std::cout << "------------------------------------------------------------" << std::endl;
              std::this_thread::sleep_for(std::chrono::seconds(1));

            }
          } else {
            RCLCPP_ERROR(node->get_logger(), "Failed to call service switch_controller");
            return 1;
          }
        }
        switch_request->deactivate_controllers.clear();
        state = 0;
        break;
      }
    }

    // Create and publish a RobotFreedriveState message
    freedrive_state_msg.state = old_state;
    if(old_state == 1){
      freedrive_state_msg.message = "FREEDRIVE";
      robot_state_pub->publish(freedrive_state_msg);
    }else if (old_state == 2){
      freedrive_state_msg.message = "RUNNING";
      robot_state_pub->publish(freedrive_state_msg);
    }else{
      RCLCPP_WARN(node->get_logger(), "Robot not ready!");
    }

    rclcpp::spin_some(node);
  }

  rclcpp::shutdown();
  return 0;
}
