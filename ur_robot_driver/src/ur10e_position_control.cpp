#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "ur_dashboard_msgs/msg/robot_freedrive_state.hpp"

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
        joint_command_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/forward_position_controller/commands", 10);
        ik_pose_error_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/ik_pose_error", 10);
        ik_output_pose_des_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("ik_output_pose_des", 10);
        
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

        freedrive_status_sub_ = this->create_subscription<ur_dashboard_msgs::msg::RobotFreedriveState>(
            "/freedrive_node/freedrive_state", 10,
            std::bind(&UrPositionControl::freedriveStatusCallback, this, std::placeholders::_1)
        );

        publish_pose_error_service_ = this->create_service<std_srvs::srv::SetBool>(
            "publish_pose_error",  
            std::bind(&UrPositionControl::publishPoseError, this, std::placeholders::_1, std::placeholders::_2)
        );

        publish_ik_output_pose_service_ = this->create_service<std_srvs::srv::SetBool>(
            "publish_ik_output_pose",  
            std::bind(&UrPositionControl::publishIkOutputPose, this, std::placeholders::_1, std::placeholders::_2)
        );

        ik_activation_service = this->create_service<std_srvs::srv::SetBool>(
            "activate_inverse_kinematic",  
            std::bind(&UrPositionControl::activateInverseKinematic, this, std::placeholders::_1, std::placeholders::_2)
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
                    RCLCPP_INFO(this->get_logger(), "Node state: %s", "IDLE");
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
                    RCLCPP_INFO(this->get_logger(), "Node state: %s", "INIT");

                    // Stop the info prompt in the run function
                    old_state_ = state_;
                }


                // The init function is executed only if the position control
                // received the first joint state message from the robot
                if(joint_state_obtained_)
                {
                    init();

                    if(freedrive_on_ || !ik_active_){
                        state_ = 1;
                    }else{
                        state_ = 2;
                    }
                }
                break;
            }
            case 2:
            {
                if(state_ != old_state_)
                {
                    RCLCPP_INFO(this->get_logger(), "Node state: %s", "UPDATE");
                    // Stop the info prompt in the run function
                    old_state_ = state_;
                }

                update();

                if(freedrive_on_ || !ik_active_){
                    state_ = 1;   
                }

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
        std::tie(joint_command_, ik_pose_error_)  = ur_kin_->ComputeInverseKinematic(desired_pos_,desired_quat_);

        if(publish_pose_error_){
            ik_pose_error_msg_.data.clear();
            for(unsigned int i=0; i < ik_pose_error_.rows(); i++){
                ik_pose_error_msg_.data.push_back(ik_pose_error_(i));
            }
            ik_pose_error_pub_->publish(ik_pose_error_msg_);
        }

        if(check_ik_output_){
            auto result = ur_kin_->ComputeDirectKinematic(joint_command_);
            ik_output_pose_msg = eigen_pose_to_pose_stamped(result.first, result.second, this->get_clock()->now(), "wrist_3_link");
            ik_output_pose_des_pub_->publish(ik_output_pose_msg);
        }

        for(unsigned int i=0; i < joint_command_.rows(); i++){
            joint_command_msg_.data.push_back(joint_command_(i));
        }
        joint_command_pub_->publish(joint_command_msg_);
    }

private:

    // Functional variables
    int state_ = 0;
    int old_state_ = -1;

    bool first_spin_ = true;
    bool freedrive_on_ = false;
    bool check_ik_output_ = false; // true: publish inverse kinematic output pose 
    bool publish_pose_error_ = false; // true: publish pose_error between desired pose and current pose
    bool ik_active_ = true; // true: publish pose_error between desired pose and current pose
    bool joint_state_obtained_ = false;
    bool pose_des_update_flag_ = true;

    double rate_;

    // KDL Variables
    KDL::JntArray           joint_command_;
    Eigen::VectorXd         ik_pose_error_;

    // ROS2 Messages
    std_msgs::msg::Float64MultiArray ik_pose_error_msg_;
    std_msgs::msg::Float64MultiArray joint_command_msg_;
    geometry_msgs::msg::PoseStamped current_pose_msg_;
    geometry_msgs::msg::PoseStamped ik_output_pose_msg;

    // ROS2 Publisher
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr current_pose_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr ik_output_pose_des_pub_;

    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr ik_pose_error_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr joint_command_pub_;

    // ROS2 Subscribers
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_des_sub_;
    rclcpp::Subscription<ur_dashboard_msgs::msg::RobotFreedriveState>::SharedPtr freedrive_status_sub_;

    // ROS" Server service
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr publish_pose_error_service_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr publish_ik_output_pose_service_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr ik_activation_service;

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
            pose_des_update_flag_ = true;
            std::tie(desired_pos_, desired_quat_) = pose_stamped_to_eigen_pose(*pose_desired_msg);
        }
        else if(pose_des_update_flag_)
        {
            RCLCPP_WARN(this->get_logger(), "Desired pose callback is deactivated!");
            pose_des_update_flag_ = false;   
        }
    }

    // Callback function for desired pose
    void freedriveStatusCallback(ur_dashboard_msgs::msg::RobotFreedriveState::UniquePtr freedrive_status_msg_)
    {
        if(freedrive_status_msg_->state == 1){
            freedrive_on_ = true;
            RCLCPP_WARN(this->get_logger(), "Freedrive activate! Cannot send reference to the position controller");
        }else if (freedrive_status_msg_->state == 2){
            freedrive_on_ = false;
        }
    }

    void activateInverseKinematic(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                       std::shared_ptr<std_srvs::srv::SetBool::Response> response) 
    {
        ik_active_ = request->data; // true: publish inverse kinematic output pose 
        if(ik_active_){
            RCLCPP_WARN(this->get_logger(), "Inverse kinematic activated! Robot is about to move!");
        }else{
            RCLCPP_WARN(this->get_logger(), "Inverse kinematic stopped!");
        }
    }

    // Callback function for desired pose
    void publishPoseError(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                       std::shared_ptr<std_srvs::srv::SetBool::Response> response)
    {
        publish_pose_error_ = request->data; // true: publish inverse kinematic output pose 
        if(publish_pose_error_){
            RCLCPP_WARN(this->get_logger(), "Publishing pose error between desired pose and current pose");
        }else{
            RCLCPP_WARN(this->get_logger(), "Pose error publisher stopped.");
        }
    }

    // Callback function for desired pose
    void publishIkOutputPose(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                       std::shared_ptr<std_srvs::srv::SetBool::Response> response)
    {
        check_ik_output_ = request->data;
        if(check_ik_output_){
            RCLCPP_WARN(this->get_logger(), "Publishing inverse kinematic output desired pose for debug");
        }else{
            RCLCPP_WARN(this->get_logger(), "Inverse kinematic output desired pose publisher stopped.");
        }
    }

};


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    double rate(300.0);
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