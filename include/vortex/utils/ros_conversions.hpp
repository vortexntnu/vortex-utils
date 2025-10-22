#ifndef ROS_CONVERSIONS_HPP
#define ROS_CONVERSIONS_HPP

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include "vortex/utils/types.hpp"
#include <ranges>
#include <vector>

namespace vortex::utils::ros_conversions {

// @brief Convert a vortex::utils::types::Pose to a ROS PoseStamped message
inline geometry_msgs::msg::PoseStamped pose_to_ros(const vortex::utils::types::Pose& pose) {
    geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.pose.position.x = pose.position(0);
    pose_stamped.pose.position.y = pose.position(1);
    pose_stamped.pose.position.z = pose.position(2);
    // Eigen stores quaternions as (w, x, y, z)
    // while ROS/tf2 uses (x, y, z, w)
    pose_stamped.pose.orientation.x = pose.orientation.x();
    pose_stamped.pose.orientation.y = pose.orientation.y();
    pose_stamped.pose.orientation.z = pose.orientation.z();
    pose_stamped.pose.orientation.w = pose.orientation.w();
    return pose_stamped;
}

// @brief Convert a ROS PoseStamped message to a vortex::utils::Pose
inline vortex::utils::types::Pose ros_to_pose(const geometry_msgs::msg::PoseStamped& pose_stamped) {
    Eigen::Vector3d position;
    position(0) = pose_stamped.pose.position.x;
    position(1) = pose_stamped.pose.position.y;
    position(2) = pose_stamped.pose.position.z;

    Eigen::Quaterniond orientation;
    // ROS/tf2 uses (x, y, z, w)
    // while Eigen stores quaternions as (w, x, y, z)
    orientation.x() = pose_stamped.pose.orientation.x;
    orientation.y() = pose_stamped.pose.orientation.y;
    orientation.z() = pose_stamped.pose.orientation.z;
    orientation.w() = pose_stamped.pose.orientation.w;

    return vortex::utils::types::Pose(position, orientation);
}

// @brief Convert a vector of vortex::utils::types::Pose to a ROS PoseArray
inline geometry_msgs::msg::PoseArray poses_to_ros(const std::vector<vortex::utils::types::Pose>& poses) {
    geometry_msgs::msg::PoseArray pose_array;
    pose_array.poses.reserve(poses.size());

    std::ranges::transform(poses, std::back_inserter(pose_array.poses),
        [](const auto& pose) {
            return pose_to_ros(pose).pose;
        });

    return pose_array;
}

// @brief Convert a ROS PoseArray to  vector<vortex::utils::types::Pose>
inline std::vector<vortex::utils::types::Pose> ros_to_poses(const geometry_msgs::msg::PoseArray& pose_array) {
    std::vector<vortex::utils::types::Pose> poses;
    poses.reserve(pose_array.poses.size());

    std::ranges::transform(pose_array.poses, std::back_inserter(poses),
        [](const geometry_msgs::msg::Pose& ros_pose) {
            geometry_msgs::msg::PoseStamped pose_stamped;
            pose_stamped.pose = ros_pose;
            return ros_to_pose(pose_stamped);
        });

    return poses;
}

}  // namespace vortex::utils::ros_conversions

#endif // ROS_CONVERSIONS_HPP