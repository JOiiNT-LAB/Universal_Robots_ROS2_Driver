#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/empty.hpp"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("activate_client");

  // Define the client in the proper scope
  auto client = node->create_client<std_srvs::srv::Empty>("/hardware_interface/enable_freedrive");

  while (!client->wait_for_service(std::chrono::seconds(1)))
  {
    RCLCPP_INFO(node->get_logger(), "Waiting for service to become available...");
  }

  auto request = std::make_shared<std_srvs::srv::Empty::Request>();
  auto result = client->async_send_request(request);

  // Using node directly instead of shared_from_this()
  if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_INFO(node->get_logger(), "Service call succeeded.");
  }
  else
  {
    RCLCPP_ERROR(node->get_logger(), "Service call failed.");
  }

  rclcpp::shutdown();
  return 0;
}
