#ifndef VORTEX_UTILS_ROS__WAYPOINT_ROS_CONVERSIONS_HPP_
#define VORTEX_UTILS_ROS__WAYPOINT_ROS_CONVERSIONS_HPP_

#include <geometry_msgs/msg/pose.hpp>
#include <vortex/utils/ros/ros_conversions.hpp>
#include <vortex/utils/types.hpp>
#include <vortex/utils/waypoint_utils.hpp>
#include <vortex_msgs/msg/waypoint.hpp>

namespace vortex::utils::waypoints {

/**
 * @brief Convert a ROS waypoint mode to a WaypointMode enum.
 * @throws std::invalid_argument if the mode value is not recognized.
 */
inline WaypointMode waypoint_mode_from_ros(uint8_t mode) {
    switch (mode) {
        case vortex_msgs::msg::Waypoint::FULL_POSE:
            return WaypointMode::FULL_POSE;
        case vortex_msgs::msg::Waypoint::ONLY_POSITION:
            return WaypointMode::ONLY_POSITION;
        case vortex_msgs::msg::Waypoint::FORWARD_HEADING:
            return WaypointMode::FORWARD_HEADING;
        case vortex_msgs::msg::Waypoint::ONLY_ORIENTATION:
            return WaypointMode::ONLY_ORIENTATION;
        default:
            throw std::invalid_argument("Invalid ROS waypoint mode: " +
                                        std::to_string(mode));
    }
}

/**
 * @brief Convert a ROS Waypoint message to an internal Waypoint struct.
 */
inline vortex::utils::types::Waypoint waypoint_from_ros(
    const vortex_msgs::msg::Waypoint& ros_wp) {
    vortex::utils::types::Waypoint wp;
    wp.pose = vortex::utils::ros_conversions::ros_pose_to_pose(ros_wp.pose);
    wp.mode = waypoint_mode_from_ros(ros_wp.mode);
    return wp;
}

}  // namespace vortex::utils::waypoints

#endif  // VORTEX_UTILS_ROS__WAYPOINT_ROS_CONVERSIONS_HPP_
