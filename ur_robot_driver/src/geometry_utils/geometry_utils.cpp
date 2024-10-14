#include "geometry_utils/geometry_utils.hpp" // Ensure this header is properly included


std::pair<Eigen::Vector3d, Eigen::Quaterniond> pose_stamped_to_eigen_pose(const geometry_msgs::msg::PoseStamped& pose_stamped)
{
    // Crea un vettore per la posizione
    Eigen::Vector3d position;
    position.x() = pose_stamped.pose.position.x;
    position.y() = pose_stamped.pose.position.y;
    position.z() = pose_stamped.pose.position.z;

    // Crea un quaternione per l'orientamento
    Eigen::Quaterniond quaternion;
    quaternion.x() = pose_stamped.pose.orientation.x;
    quaternion.y() = pose_stamped.pose.orientation.y;
    quaternion.z() = pose_stamped.pose.orientation.z;
    quaternion.w() = pose_stamped.pose.orientation.w;

    // Restituisci un std::pair con posizione e quaternione
    return std::make_pair(position, quaternion);
}

geometry_msgs::msg::PoseStamped eigen_pose_to_pose_stamped(const Eigen::Vector3d& position, 
                                                      const Eigen::Quaterniond& quaternion, 
                                                      const rclcpp::Time timestamp,
                                                      const std::string& frame_id) 
{
    geometry_msgs::msg::PoseStamped pose_stamped;

    // Set the header (frame ID and timestamp)
    pose_stamped.header.frame_id = frame_id;
    pose_stamped.header.stamp = timestamp;

    // Set the position
    pose_stamped.pose.position.x = position.x();
    pose_stamped.pose.position.y = position.y();
    pose_stamped.pose.position.z = position.z();

    // Set the orientation
    pose_stamped.pose.orientation.x = quaternion.x();
    pose_stamped.pose.orientation.y = quaternion.y();
    pose_stamped.pose.orientation.z = quaternion.z();
    pose_stamped.pose.orientation.w = quaternion.w();

    return pose_stamped;
}

