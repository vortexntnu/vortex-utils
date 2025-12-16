#ifndef VORTEX_UTILS__VIEWS_HPP_
#define VORTEX_UTILS__VIEWS_HPP_

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include "accessors.hpp"
#include "concepts.hpp"

namespace vortex::utils::views {

using vortex::utils::concepts::EulerLike;
using vortex::utils::concepts::EulerPoseLike;
using vortex::utils::concepts::PositionLike;
using vortex::utils::concepts::QuaternionLike;
using vortex::utils::concepts::QuatPoseLike;

using vortex::utils::types::x_of;
using vortex::utils::types::y_of;
using vortex::utils::types::z_of;

using vortex::utils::types::qw_of;
using vortex::utils::types::qx_of;
using vortex::utils::types::qy_of;
using vortex::utils::types::qz_of;

using vortex::utils::types::pitch_of;
using vortex::utils::types::roll_of;
using vortex::utils::types::yaw_of;

template <PositionLike T>
inline Eigen::Vector3d pos_vector(const T& t) {
    return Eigen::Vector3d{x_of(t), y_of(t), z_of(t)};
}

template <EulerLike T>
inline Eigen::Vector3d ori_vector(const T& t) {
    return Eigen::Vector3d{roll_of(t), pitch_of(t), yaw_of(t)};
}

template <QuaternionLike T>
inline Eigen::Quaterniond ori_quaternion(const T& t) {
    return Eigen::Quaterniond{qw_of(t), qx_of(t), qy_of(t), qz_of(t)};
}

template <EulerPoseLike T>
inline Eigen::Vector<double, 6> to_vector(const T& t) {
    return Eigen::Vector<double, 6>{x_of(t),    y_of(t),     z_of(t),
                                    roll_of(t), pitch_of(t), yaw_of(t)};
}

template <QuatPoseLike T>
inline Eigen::Vector<double, 7> to_vector(const T& t) {
    return Eigen::Vector<double, 7>{x_of(t),  y_of(t),  z_of(t), qw_of(t),
                                    qx_of(t), qy_of(t), qz_of(t)};
}

}  // namespace vortex::utils::views

#endif  // VORTEX_UTILS__VIEWS_HPP_
