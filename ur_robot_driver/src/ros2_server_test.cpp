#include <rclcpp/rclcpp.hpp>
#include <ur_client_library/ur/ur_driver.h>
#include "ur_client_library/control/reverse_interface.h"

class MyNode : public rclcpp::Node {
public:
  MyNode() : Node("my_node") {
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&MyNode::timerCallback, this)
    );
  }

private:
  void timerCallback() {
    RCLCPP_INFO(this->get_logger(), "Timer callback");
  }

  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("example");
  // std::unique_ptr<UrDriver> ur_driver_;

  // const std::string DEFAULT_ROBOT_IP = "192.168.56.100";
  // const std::string SCRIPT_FILE = "/home/arise/freedrive_gianluca_UR/ur/Universal_Robots_Client_Library/resources/external_control.urscript";
  // const std::string OUTPUT_RECIPE = "/home/arise/freedrive_gianluca_UR/ur/Universal_Robots_Client_Library/examples/resources/rtde_output_recipe.txt";
  // const std::string INPUT_RECIPE = "/home/arise/freedrive_gianluca_UR/ur/Universal_Robots_Client_Library/examples/resources/freedrive.txt";
  // const std::string CALIBRATION_CHECKSUM = "calib_12788084448423163542";

  // ur_driver_->startRTDECommunication();
  // while (true)
  // {
  //   // Read latest RTDE package. This will block for a hard-coded timeout (see UrDriver), so the
  //   // robot will effectively be in charge of setting the frequency of this loop.
  //   // In a real-world application this thread should be scheduled with real-time priority in order
  //   // to ensure that this is called in time.
  //   ur_driver_->writeFreedriveControlMessage(urcl::control::FreedriveControlMessage::FREEDRIVE_START);
  // }

  rclcpp::shutdown();
  return 0;
}