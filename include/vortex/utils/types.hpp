#ifndef VORTEX_UTILS__POSE_TYPES_HPP_
#define VORTEX_UTILS__POSE_TYPES_HPP_

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

namespace vortex::utils::types {

/**
 * @brief Struct to represent the state vector eta according to eq. 2.3 in
 * Fossen, 2021, containing the position and orientation.
 */
struct Eta {
    double x{};
    double y{};
    double z{};
    double roll{};
    double pitch{};
    double yaw{};
};

/**
 * @brief Struct to represent the state vector eta according to eq. 2.3 in
 * Fossen, 2021, containing the position and orientation as quaternion.
 */
struct EtaQuat {
    double x{};
    double y{};
    double z{};
    double qw{1.0};
    double qx{};
    double qy{};
    double qz{};
};

/**
 * @brief Eigen-based pose, for interfaces / measurements
 */
struct PoseQuatEigen {
    Eigen::Vector3d position;
    Eigen::Quaterniond orientation;
};

/**
 * @brief Struct to represent the state vector nu according to eq. 2.5 in
 * Fossen, 2021, containing the linear and angular velocities.
 */
struct Nu {
    double u{};
    double v{};
    double w{};
    double p{};
    double q{};
    double r{};
};

}  // namespace vortex::utils::types

#endif  // VORTEX_UTILS__POSE_TYPES_HPP_
