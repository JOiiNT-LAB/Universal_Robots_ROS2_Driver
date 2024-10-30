#include <control_msgs/JointControllerState.h> 

#include <urdf/model.h>
#include <controller_interface/controller.h>
#include <ros/node_handle.h>
#include <ros/ros.h>

#include <kdl/tree.hpp>
#include <kdl/kdl.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/frames.hpp>
#include <kdl/chaindynparam.hpp> //this to compute the gravity vector
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <eigen3/Eigen/Eigen>
#include <skew_symmetric.h>
#include <pseudo_inversion.h>

#include <boost/scoped_ptr.hpp>
#include <boost/thread/condition.hpp>

#include <sensor_msgs/Joy.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>

#include <std_msgs/Float64MultiArray.h>
#include <std_srvs/Empty.h>



class OneTaskInvKin
{
	public:
		OneTaskInvKin();
		~OneTaskInvKin();

		void run();
		double dt_;

	private:

		void callback_joint_states(const sensor_msgs::JointState::ConstPtr& msg);
		void callback_des_pose(const geometry_msgs::Pose::ConstPtr& msg);
		bool callback_inv_kin(std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res);
		void callback_des_pose_stamped(const geometry_msgs::PoseStamped::ConstPtr& msg);

		void init();
		void update();


	
		ros::NodeHandle n_;
		ros::Subscriber sub_joint_states_, sub_des_pos_;
		ros::Publisher pub_joint_cmd_, current_pose_pub, pub_joint_, pub_orient_error_;
		ros::ServiceServer server_inv_kin_;

		KDL::Chain kdl_chain_;

		KDL::JntArray  q_msr_;          // Joint measured positions
		KDL::JntArray  q_msr_out;       // Joint measured positions, for cartesian pose reading only
		KDL::JntArray  q_;        		// Joint computed positions
		KDL::Jacobian  J_;            // Jacobian
		KDL::Frame     x_;            // Tip pose    
		KDL::Frame     x_out;            // Tip pose, read only                                                                                                                                       

		boost::scoped_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_solver_;
		boost::scoped_ptr<KDL::ChainFkSolverPos_recursive> fk_pos_solver_;

		Eigen::VectorXd joint_location_, q_eig_;
		Eigen::Vector3d pos_d_;
		Eigen::Quaterniond quat_d_, quat_old_;
		Eigen::MatrixXd k_;
		int step_, sign_curr_des_;
		bool flag_joint_msr_, first_quat_, first_des_;
		Eigen::MatrixXd W_;

		// counter timeout to check the correct reception of the joint states
		int count_joint_states_missing = 0 ;
		const int limit_joint_state_missing = 2 ;


};