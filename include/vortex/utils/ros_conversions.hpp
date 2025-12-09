#ifndef ROS_CONVERSIONS_HPP
#define ROS_CONVERSIONS_HPP

#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace vortex::utils::ros_conversions {

template <typename T>
geometry_msgs::msg::Pose reference_to_pose(const T& ref) {
    geometry_msgs::msg::Pose pose;
    pose.position.x = ref.x;
    pose.position.y = ref.y;
    pose.position.z = ref.z;

    tf2::Quaternion q;
    q.setRPY(ref.roll, ref.pitch, ref.yaw);
    pose.orientation = tf2::toMsg(q);

    return pose;
}

}  // namespace vortex::utils::ros_conversions

#endif  // ROS_CONVERSIONS_HPP
