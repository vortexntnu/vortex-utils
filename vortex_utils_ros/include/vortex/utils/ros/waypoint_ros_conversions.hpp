#ifndef VORTEX_UTILS_ROS__WAYPOINT_ROS_CONVERSIONS_HPP_
#define VORTEX_UTILS_ROS__WAYPOINT_ROS_CONVERSIONS_HPP_

#include <geometry_msgs/msg/pose.hpp>
#include <vortex/utils/ros/ros_conversions.hpp>
#include <vortex/utils/types.hpp>
#include <vortex/utils/waypoint_utils.hpp>
#include <vortex_msgs/msg/waypoint.hpp>
#include <vortex_msgs/msg/waypoint_mode.hpp>

namespace vortex::utils::waypoints {

/**
 * @brief Convert a ROS waypoint mode to a WaypointMode enum.
 * @throws std::invalid_argument if the mode value is not recognized.
 */
inline WaypointMode waypoint_mode_from_ros(
    const vortex_msgs::msg::WaypointMode& mode_msg) {
    switch (mode_msg.mode) {
        case vortex_msgs::msg::WaypointMode::FULL_POSE:
            return WaypointMode::FULL_POSE;
        case vortex_msgs::msg::WaypointMode::ONLY_POSITION:
            return WaypointMode::ONLY_POSITION;
        case vortex_msgs::msg::WaypointMode::FORWARD_HEADING:
            return WaypointMode::FORWARD_HEADING;
        case vortex_msgs::msg::WaypointMode::ONLY_ORIENTATION:
            return WaypointMode::ONLY_ORIENTATION;
        default:
            throw std::invalid_argument("Invalid ROS waypoint mode: " +
                                        std::to_string(mode_msg.mode));
    }
}

/**
 * @brief Convert a WaypointMode enum to a ROS waypoint mode message.
 */
inline vortex_msgs::msg::WaypointMode waypoint_mode_to_ros(
    const WaypointMode& mode_msg) {
    vortex_msgs::msg::WaypointMode ros_mode;
    switch (mode_msg) {
        case WaypointMode::FULL_POSE:
            ros_mode.mode = vortex_msgs::msg::WaypointMode::FULL_POSE;
            break;
        case WaypointMode::ONLY_POSITION:
            ros_mode.mode = vortex_msgs::msg::WaypointMode::ONLY_POSITION;
            break;
        case WaypointMode::FORWARD_HEADING:
            ros_mode.mode = vortex_msgs::msg::WaypointMode::FORWARD_HEADING;
            break;
        case WaypointMode::ONLY_ORIENTATION:
            ros_mode.mode = vortex_msgs::msg::WaypointMode::ONLY_ORIENTATION;
            break;
    }
    return ros_mode;
}

/**
 * @brief Convert a ROS Waypoint message to an internal Waypoint struct.
 */
inline vortex::utils::types::Waypoint waypoint_from_ros(
    const vortex_msgs::msg::Waypoint& ros_wp) {
    vortex::utils::types::Waypoint wp;
    wp.pose = vortex::utils::ros_conversions::ros_pose_to_pose(ros_wp.pose);
    wp.mode = waypoint_mode_from_ros(ros_wp.waypoint_mode);
    return wp;
}

}  // namespace vortex::utils::waypoints

#endif  // VORTEX_UTILS_ROS__WAYPOINT_ROS_CONVERSIONS_HPP_
