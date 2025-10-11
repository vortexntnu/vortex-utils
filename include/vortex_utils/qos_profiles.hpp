#ifndef VORTEX_UTILS_QOS_PROFILES_HPP
#define VORTEX_UTILS_QOS_PROFILES_HPP

#include <rclcpp/qos.hpp>

namespace vortex::utils::qos_profiles {

inline rclcpp::QoS sensor_data_profile(const size_t depth = 5) {
    auto profile{rclcpp::QoS(depth)};  // Keep last
    profile.best_effort();
    profile.durability_volatile();
    return profile;
}

inline rclcpp::QoS reliable_profile(const size_t depth = 10) {
    auto profile{rclcpp::QoS(depth)};  // Keep last
    profile.reliable();
    profile.durability_volatile();
    return profile;
}

}  // namespace vortex::utils::qos_profiles

#endif  // VORTEX_UTILS_QOS_PROFILES_HPP
