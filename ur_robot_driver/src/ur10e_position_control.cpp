#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include "geometry_lib.hpp" 
#include "robot_kinematic.hpp" 

using namespace std::chrono_literals;

class UrPositionControl : public rclcpp::Node
{
public:

    UrPositionControl(double rate) : Node("ur10e_position_control"), rate_(rate)
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

        std::string             root_name = "base_link_inertia";
        std::string             tip_name = "wrist_3_link";
        
        ur_kin_ = std::make_shared<RobotKinematic>(robot_description_, root_name, tip_name, rate);

        // Publisher for the topic /robot_ur10e/current_pose
        current_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("current_pose", 10);
        test_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("test_output_pose", 10);
        joint_command_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/forward_position_controller/commands", 10);

        // Subscribe to the joint states topic
        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            std::bind(&UrPositionControl::jointStateCallback, this, std::placeholders::_1)
        );

        // Subscribe to the desired pose topic
        pose_des_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/pose_des", 10,
            std::bind(&UrPositionControl::poseDesCallback, this, std::placeholders::_1)
        );
    }


    void run()
    {
            switch (state_)
            {
                case 0:
                {
                    if(state_ != old_state_)
                    {
                        RCLCPP_INFO(this->get_logger(), "node state: %s", "IDLE");

                        // Stop the info prompt in the run function
                        old_state_ = state_;
                    }

                    state_ = 1;


                    break;
                }
                case 1:
                {
                    if(state_ != old_state_)
                    {
                        RCLCPP_INFO(this->get_logger(), "node state: %s", "INIT");

                        // Stop the info prompt in the run function
                        old_state_ = state_;
                    }


                    // The init function is executed only if the position control 
                    // received the first joint state message from the robot
                    if(joint_state_obtained_)
                    {
                        init();
                        state_ = 2;
                    }
                    break;
                }
                case 2:
                {
                    if(state_ != old_state_)
                    {
                        RCLCPP_INFO(this->get_logger(), "node state: %s", "UPDATE");
                        // Stop the info prompt in the run function
                        old_state_ = state_;
                    }
                    update();
                    break;
                }
            }
    }

    void init()
    {
        ur_kin_->Init();
        desired_pos_ = current_pos_;
        desired_quat_ = current_quat_;
    }

    void update()
    {
        joint_command_msg_.data.clear();
        joint_command_ = ur_kin_->ComputeInverseKinematic(desired_pos_,desired_quat_);
        
        joint_command_msg_.data.push_back(joint_command_(0));
        joint_command_msg_.data.push_back(joint_command_(1));
        joint_command_msg_.data.push_back(joint_command_(2));
        joint_command_msg_.data.push_back(joint_command_(3));
        joint_command_msg_.data.push_back(joint_command_(4));
        joint_command_msg_.data.push_back(joint_command_(5));

        // for(int i = 0; i < 6; i++)
        // {
        //     std::cout << joint_command_msg_.data[i] << std::endl;
        // }

        auto result = ur_kin_->ComputeDirectKinematic(joint_command_);

        auto test_output_pose = eigen_pose_to_pose_stamped(result.first, result.second, this->get_clock()->now(), "wrist_3_link");
        test_pose_pub_->publish(test_output_pose);
        joint_command_pub_->publish(joint_command_msg_);
    }

private:

    // Functional variables
    int state_ = 0;
    int old_state_ = -1;

    bool first_spin_ = true;
    bool joint_state_obtained_ = false;

    double rate_;

    // KDL Variables
    KDL::JntArray joint_command_;

    // ROS2 Messages
    std_msgs::msg::Float64MultiArray joint_command_msg_;
    geometry_msgs::msg::PoseStamped current_pose_msg_;

    // ROS2 Publisher
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr current_pose_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr test_pose_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr joint_command_pub_;

    // ROS2 Subscribers
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_des_sub_;

    // Robot kinematic model
    Eigen::Vector3d         current_pos_;
    Eigen::Quaterniond      current_quat_;

    Eigen::Vector3d         desired_pos_;
    Eigen::Quaterniond      desired_quat_;

    std::shared_ptr<RobotKinematic> ur_kin_;

    // Callback function for joint states
    void jointStateCallback(sensor_msgs::msg::JointState::UniquePtr joint_state_msg) 
    {       
        ur_kin_->q_measured_(0) = joint_state_msg->position[5];
        ur_kin_->q_measured_(1) = joint_state_msg->position[0];
        ur_kin_->q_measured_(2) = joint_state_msg->position[1];
        ur_kin_->q_measured_(3) = joint_state_msg->position[2];
        ur_kin_->q_measured_(4) = joint_state_msg->position[3];
        ur_kin_->q_measured_(5) = joint_state_msg->position[4];

        // Compute the direct kinematics
        std::tie(current_pos_, current_quat_) = ur_kin_->ComputeDirectKinematic(ur_kin_->q_measured_);

        // Convert Eigen vector position and quaternion to geometry_msgs/PoseStamped
        current_pose_msg_ = eigen_pose_to_pose_stamped(current_pos_, current_quat_, this->get_clock()->now(), "wrist_3_link");

        // Publish the message to ROS
        current_pose_pub_->publish(current_pose_msg_);
        joint_state_obtained_ = true;
    }

    // Callback function for desired pose
    void poseDesCallback(geometry_msgs::msg::PoseStamped::UniquePtr pose_desired_msg) 
    {
        if(state_ == 2)
        {
            std::tie(desired_pos_, desired_quat_) = pose_stamped_to_eigen_pose(*pose_desired_msg);
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "the position controller is NOT subscribed to the joint states topic or has NOT received the first message");
        }
    }

};


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    double rate(100.0);
    rclcpp::Rate ros_loop_rate(rate);  // 400 Hz loop
    
    auto node = std::make_shared<UrPositionControl>(rate);

    // Create the executor
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);

    
    // Main loop mimicking ROS 1's spinOnce()
    while (rclcpp::ok())
    {
        // Process any pending callbacks (like ROS 1's spinOnce)
        node->run();
        executor.spin_some();
        
        // Sleep to maintain the loop rate
        ros_loop_rate.sleep();
    }

    
    // // ROS2 node shutdown
    rclcpp::shutdown();    
    return 0;
}

//  ######################################################################################
//     ERROR: if you have got the model from the parameter server and then get stuck in 
//     the driver crashes, the node cannot get another joint state topic    
//  ######################################################################################