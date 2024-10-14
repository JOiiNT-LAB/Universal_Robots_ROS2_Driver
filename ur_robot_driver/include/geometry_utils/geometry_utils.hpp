#ifndef GEOMETRY_UTILS_HPP
#define GEOMETRY_UTILS_HPP

#include "geometry_msgs/msg/pose_stamped.hpp"

#include <rclcpp/time.hpp>

#include <Eigen/Dense>
#include <utility>         // Per std::pair

std::pair<Eigen::Vector3d, Eigen::Quaterniond> pose_stamped_to_eigen_pose(const geometry_msgs::msg::PoseStamped& pose_stamped);

geometry_msgs::msg::PoseStamped eigen_pose_to_pose_stamped(const Eigen::Vector3d& position, 
                                                      const Eigen::Quaterniond& quaternion, 
                                                      const rclcpp::Time timestamp,
                                                      const std::string& frame_id = "");
                                                      
#endif // GEOMETRY_UTILS_HPP