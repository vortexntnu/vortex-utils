#ifndef VORTEX_UTILS__ROS_CONVERSIONS_HPP_
#define VORTEX_UTILS__ROS_CONVERSIONS_HPP_

#include "math.hpp"
#include <geometry_msgs/msg/pose.hpp>

namespace vortex::utils::ros_conversions {

template <typename T>  
concept PoseLike = requires(const T& t) {  
    { t.x }     -> std::convertible_to<double>;  
    { t.y }     -> std::convertible_to<double>;  
    { t.z }     -> std::convertible_to<double>;  
    { t.roll }  -> std::convertible_to<double>;  
    { t.pitch } -> std::convertible_to<double>;  
    { t.yaw }   -> std::convertible_to<double>;  
};  

template <PoseLike T>
geometry_msgs::msg::Pose reference_to_pose(const T& ref) {
    geometry_msgs::msg::Pose pose;
    pose.position.x = ref.x;
    pose.position.y = ref.y;
    pose.position.z = ref.z;

    Eigen::Quaterniond quat =
        vortex::utils::math::euler_to_quat(ref.roll, ref.pitch, ref.yaw);

    pose.orientation.x = quat.x();
    pose.orientation.y = quat.y();
    pose.orientation.z = quat.z();
    pose.orientation.w = quat.w();

    return pose;
}

}  // namespace vortex::utils::ros_conversions

#endif  // VORTEX_UTILS__ROS_CONVERSIONS_HPP_
