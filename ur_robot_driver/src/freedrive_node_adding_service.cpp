#include <memory>
#include <string>
#include <chrono>
#include <cstdlib>
#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "controller_manager_msgs/msg/controller_state.hpp"
#include "controller_manager_msgs/srv/list_controllers.hpp"
#include "controller_manager_msgs/srv/switch_controller.hpp"

using namespace std::chrono_literals;



int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("freedrive_node");
  auto list_controller_client_ = node->create_client<controller_manager_msgs::srv::ListControllers>("controller_manager/list_controllers");
  auto switch_controller_client_ = node->create_client<controller_manager_msgs::srv::SwitchController>("controller_manager/switch_controller");

  int state(0);
  bool new_idle_state_(false);

  // while(ROS_OK){


    // switch (state) {
    //   case 0:
    //         if(new_idle_state_){
    //           RCLCPP_INFO(node->get_logger(), "Idle state: waiting request to enable freedrive mode");
    //           new_idle_state_ = false;
    //         }
    //         break;
    //   case 1:
    //     RCLCPP_INFO(node->get_logger(), "ENABLE Request received: disabling control and launching freedrive mode");

    //     // Wait for list_controllers service to become available
    //     while (!list_controller_client_->wait_for_service(1s)) {
    //       if (!rclcpp::ok()) {
    //         RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for the service. Exiting.");
    //         return 1;
    //       }
    //       RCLCPP_INFO(node->get_logger(), "Service not available, waiting again...");
    //     }

    //     // Prepare request for list_controllers service
    //     auto list_controller_request  = std::make_shared<controller_manager_msgs::srv::ListControllers::Request>();
    //     auto switch_request           = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();

    //     // Send request and handle response
    //     auto controller_list = list_controller_client_->async_send_request(list_controller_request);
    //     if (rclcpp::spin_until_future_complete(node, controller_list) == rclcpp::FutureReturnCode::SUCCESS) {
    //       auto response = controller_list.get();
    //       if (!response) {
    //         RCLCPP_ERROR(node->get_logger(), "Received null response from service.");
    //         return 1;
    //       }

    //       std::cout << "Listing controllers:\n";

    //       // Add all the controllers apart from joint_state_broadcaster to the switch controller request
    //       for (const auto& controller : response->controller) {
    //         if(controller.name == "joint_state_broadcaster"){
    //           RCLCPP_WARN(node->get_logger(), "Skipping deactivation of joint_state_broadcaster");  
    //           continue;
    //         }
    //         RCLCPP_INFO(node->get_logger(), "Name: %s", controller.name.c_str());
    //         RCLCPP_INFO(node->get_logger(), "State: %s", controller.state.c_str());
    //         RCLCPP_INFO(node->get_logger(), "Type: %s", controller.type.c_str());

    //         // If the controller is active, prepare to stop it
    //         if (controller.state == "active") {
    //           switch_request->deactivate_controllers.push_back(controller.name);  // Usa deactivate_controllers
    //           switch_request->strictness = controller_manager_msgs::srv::SwitchController::Request::BEST_EFFORT; 
    //         }
    //       }

    //       // Send switch_controller request to deactivate everything apart joint_state_broadcaster
    //       if(switch_request->deactivate_controllers.size() != 0){
    //         auto switch_response_future = switch_controller_client_->async_send_request(switch_request);
    //         if (rclcpp::spin_until_future_complete(node, switch_response_future) == rclcpp::FutureReturnCode::SUCCESS) {
    //           auto switch_response = switch_response_future.get();
    //           if (switch_response->ok) {
    //             RCLCPP_INFO(node->get_logger(), "Successfully switched controllers.");
    //           } else {
    //             RCLCPP_ERROR(node->get_logger(), "Failed to switch controllers.");
    //           }
    //         } else {
    //           RCLCPP_ERROR(node->get_logger(), "Failed to call service switch_controller");
    //           return 1;
    //         }
    //       } else {
    //         RCLCPP_WARN(node->get_logger(), "switch_request has no controller to deactivate!");  
    //       }

    //       // chiama servizio attivazione freedrive
    //     } else {
    //       RCLCPP_ERROR(node->get_logger(), "Failed to call service list_controllers");
    //       return 1;
    //     }

    //     break;

    //   case 2:
    //     RCLCPP_INFO(node->get_logger(), "DISABLE Request received: disabling freedrive mode, enabling back the controllers");

    //     break;
    // }

  // }

  rclcpp::shutdown();
  return 0;
}
