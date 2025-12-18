#ifndef VORTEX_UTILS__ACCESSORS_HPP_
#define VORTEX_UTILS__ACCESSORS_HPP_

#include "member_concepts.hpp"

namespace vortex::utils {

template <concepts::HasPositionMembers T>
constexpr double x_of(const T& t) {
    return t.x;
}

template <concepts::HasPositionMembers T>
constexpr double y_of(const T& t) {
    return t.y;
}

template <concepts::HasPositionMembers T>
constexpr double z_of(const T& t) {
    return t.z;
}

template <concepts::HasEulerMembers T>
constexpr double roll_of(const T& t) {
    return t.roll;
}

template <concepts::HasEulerMembers T>
constexpr double pitch_of(const T& t) {
    return t.pitch;
}

template <concepts::HasEulerMembers T>
constexpr double yaw_of(const T& t) {
    return t.yaw;
}

template <concepts::HasQuaternionMembers T>
constexpr double qw_of(const T& q) {
    return q.qw;
}

template <concepts::HasQuaternionMembers T>
constexpr double qx_of(const T& q) {
    return q.qx;
}

template <concepts::HasQuaternionMembers T>
constexpr double qy_of(const T& q) {
    return q.qy;
}

template <concepts::HasQuaternionMembers T>
constexpr double qz_of(const T& q) {
    return q.qz;
}

}  // namespace vortex::utils

#endif  // VORTEX_UTILS__ACCESSORS_HPP_
