#ifndef VORTEX_UTILS__MEMBER_CONCEPTS_HPP_
#define VORTEX_UTILS__MEMBER_CONCEPTS_HPP_

#include <concepts>

namespace vortex::utils::concepts {

template <typename T>
concept HasPositionMembers = requires(const T& t) {
    { t.x } -> std::convertible_to<double>;
    { t.y } -> std::convertible_to<double>;
    { t.z } -> std::convertible_to<double>;
};

template <typename T>
concept HasQuaternionMembers = requires(const T& t) {
    { t.qw } -> std::convertible_to<double>;
    { t.qx } -> std::convertible_to<double>;
    { t.qy } -> std::convertible_to<double>;
    { t.qz } -> std::convertible_to<double>;
};

template <typename T>
concept HasEulerMembers = requires(const T& t) {
    { t.roll } -> std::convertible_to<double>;
    { t.pitch } -> std::convertible_to<double>;
    { t.yaw } -> std::convertible_to<double>;
};

}  // namespace vortex::utils::concepts

#endif  // VORTEX_UTILS__MEMBER_CONCEPTS_HPP_
