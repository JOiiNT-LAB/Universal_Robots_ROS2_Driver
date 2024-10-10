#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "robot_kinematic.hpp" // Ensure this header is properly included

using namespace std::chrono_literals;

class UrPositionControl : public rclcpp::Node
{
public:
    UrPositionControl() : Node("ur10e_position_control")
    {
        auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(this, "robot_state_publisher");
        while (!parameters_client->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                rclcpp::shutdown();
            }
            RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
        }
        
        // Retrieve the parameter value
        auto description_parameter = parameters_client->get_parameters({"robot_description"});
        std::string robot_description_ = description_parameter[0].as_string();
        std::string root_name = "base_link_inertia";
        std::string tip_name = "wrist_3_link";
        
        ur_kin_ = std::make_shared<RobotKinematic>(robot_description_, root_name, tip_name);

        // Publisher for the topic /robot_ur10e/current_pose
        current_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/robot_ur10e/current_pose", 10);

        // Subscriber for the topic /joint_states
        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            [this](sensor_msgs::msg::JointState::UniquePtr joint_state_msg) {
                
                // Copia le posizioni delle giunture
                for (size_t i = 0; i < ur_kin_->q_measured_.rows(); i++) {
                    std::cout << joint_state_msg->name[i]  << std::endl;
                    ur_kin_->q_measured_(i) = joint_state_msg->position[i];
                }

                std::cout << "----------------------------------------" << std::endl;
                ur_kin_->q_measured_(0) = joint_state_msg->position[5];
                ur_kin_->q_measured_(1) = joint_state_msg->position[0];
                ur_kin_->q_measured_(2) = joint_state_msg->position[1];
                ur_kin_->q_measured_(3) = joint_state_msg->position[2];
                ur_kin_->q_measured_(4) = joint_state_msg->position[3];
                ur_kin_->q_measured_(5) = joint_state_msg->position[4];

                // Calcola la cinematica diretta
                auto [pos, quat] = ur_kin_->ComputeDirectKinematic(ur_kin_->q_measured_);

                // Prepara il messaggio di posa
                auto pose_msg = geometry_msgs::msg::PoseStamped();
                pose_msg.header.stamp = this->get_clock()->now();
                pose_msg.header.frame_id = "wrist_3_link";

                pose_msg.pose.position.x = pos(0);
                pose_msg.pose.position.y = pos(1);
                pose_msg.pose.position.z = pos(2);
                pose_msg.pose.orientation.x = quat.x();
                pose_msg.pose.orientation.y = quat.y();
                pose_msg.pose.orientation.z = quat.z();
                pose_msg.pose.orientation.w = quat.w();

                // Pubblica il messaggio ROS
                current_pose_pub_->publish(pose_msg);
            });
    }

private:
    // ROS2 Publisher
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr current_pose_pub_;

    // ROS2 Subscribers
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;

    // Kinematic model
    std::shared_ptr<RobotKinematic> ur_kin_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<UrPositionControl>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
