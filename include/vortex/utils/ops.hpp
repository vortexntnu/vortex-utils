#ifndef VORTEX_UTILS__OPS_HPP_
#define VORTEX_UTILS__OPS_HPP_

#include "math.hpp"
#include "types.hpp"
#include "views.hpp"

namespace vortex::utils::types {

inline EtaQuat eta_error(const EtaQuat& a, const EtaQuat& b) {
    EtaQuat out{};
    out.x = a.x - b.x;
    out.y = a.y - b.y;
    out.z = a.z - b.z;

    const Eigen::Quaterniond qa =
        vortex::utils::views::ori_quaternion(a).normalized();
    const Eigen::Quaterniond qb =
        vortex::utils::views::ori_quaternion(b).normalized();
    Eigen::Quaterniond q_error = qb.conjugate() * qa;
    q_error.normalize();

    out.qw = q_error.w();
    out.qx = q_error.x();
    out.qy = q_error.y();
    out.qz = q_error.z();
    return out;
}

inline Eta eta_error(const Eta& a, const Eta& b) {
    Eta out{};
    out.x = a.x - b.x;
    out.y = a.y - b.y;
    out.z = a.z - b.z;

    out.roll = a.roll - b.roll;
    out.pitch = a.pitch - b.pitch;
    out.yaw = a.yaw - b.yaw;

    return out;
}

/**
 * @brief Apply smallest signed angle to roll, pitch, and yaw.
 */
inline void apply_ssa(Eta& e) {
    using vortex::utils::math::ssa;
    e.roll = ssa(e.roll);
    e.pitch = ssa(e.pitch);
    e.yaw = ssa(e.yaw);
}

/**
 * @brief Make the rotation matrix corresponding to eq. 2.31 in Fossen, 2021
 * @return Eigen::Matrix3d rotation matrix
 */
Eigen::Matrix3d as_rotation_matrix(const Eta& eta) {
    return vortex::utils::math::get_rotation_matrix(eta.roll, eta.pitch,
                                                    eta.yaw);
}

/**
 * @brief Make the transformation matrix according to eq. 2.41 in Fossen,
 * 2021
 * @return Eigen::Matrix3d transformation matrix
 */
Eigen::Matrix3d as_transformation_matrix(const Eta& eta) {
    double cphi = cos(eta.roll);
    double sphi = sin(eta.roll);
    double ctheta = cos(eta.pitch);
    double stheta = sin(eta.pitch);

    if (ctheta == 0) {
        throw std::runtime_error("Division by zero in transformation matrix.");
    }

    double t11 = 1;
    double t12 = sphi * stheta / ctheta;
    double t13 = cphi * stheta / ctheta;
    double t21 = 0;
    double t22 = cphi;
    double t23 = -sphi;
    double t31 = 0;
    double t32 = sphi / ctheta;
    double t33 = cphi / ctheta;

    Eigen::Matrix3d transformation_matrix;

    transformation_matrix << t11, t12, t13, t21, t22, t23, t31, t32, t33;

    return transformation_matrix;
}

/**
 * @brief Make the J matrix according to eq. 2.53 in Fossen, 2021
 * @return Eigen::Matrix6d J matrix
 */
Eigen::Matrix<double, 6, 6> as_j_matrix(const Eta& eta) {
    Eigen::Matrix3d rotation_matrix = as_rotation_matrix(eta);
    Eigen::Matrix3d transformation_matrix = as_transformation_matrix(eta);

    Eigen::Matrix<double, 6, 6> j_matrix = Eigen::Matrix<double, 6, 6>::Zero();
    j_matrix.topLeftCorner<3, 3>() = rotation_matrix;
    j_matrix.bottomRightCorner<3, 3>() = transformation_matrix;

    return j_matrix;
}

/**
 * @brief Convert to Eigen::Vector6d
 * @return Eigen::Vector6d{x, y, z, roll, pitch, yaw}
 */
Eigen::Vector<double, 6> to_vector(const Eta& eta) {
    return Eigen::Vector<double, 6>{eta.x,    eta.y,     eta.z,
                                    eta.roll, eta.pitch, eta.yaw};
}

/**
 * @brief Convert to Eta with Euler angles
 * @return Eta
 */
inline EtaQuat as_eta_quat(const Eta& eta) {
    Eigen::Quaterniond quat =
        vortex::utils::math::euler_to_quat(eta.roll, eta.pitch, eta.yaw);

    EtaQuat eta_quat{.x = eta.x,
                     .y = eta.y,
                     .z = eta.z,
                     .qw = quat.w(),
                     .qx = quat.x(),
                     .qy = quat.y(),
                     .qz = quat.z()};
    return eta_quat;
}

/**
 * @brief Make the rotation matrix corresponding to eq. 2.31 in Fossen, 2021
 * @return Eigen::Matrix3d rotation matrix
 */
Eigen::Matrix3d as_rotation_matrix(const EtaQuat& eta) {
    return vortex::utils::views::ori_quaternion(eta)
        .normalized()
        .toRotationMatrix();
}

/**
 * @brief Make the transformation matrix according to eq. 2.78 in Fossen,
 * 2021
 * @return Eigen::Matrix<double, 4, 3> transformation matrix
 */
Eigen::Matrix<double, 4, 3> as_transformation_matrix(const EtaQuat& eta) {
    return vortex::utils::math::get_transformation_matrix_attitude_quat(
        vortex::utils::views::ori_quaternion(eta).normalized());
}

/**
 * @brief Make the J matrix according to eq. 2.83 in Fossen, 2021
 * @return Eigen::Matrix<double, 7, 6> J matrix
 */
Eigen::Matrix<double, 7, 6> as_j_matrix(const EtaQuat& eta) {
    Eigen::Matrix3d R = as_rotation_matrix(eta);
    Eigen::Matrix<double, 4, 3> T = as_transformation_matrix(eta);

    Eigen::Matrix<double, 7, 6> j_matrix = Eigen::Matrix<double, 7, 6>::Zero();
    j_matrix.topLeftCorner<3, 3>() = R;
    j_matrix.bottomRightCorner<4, 3>() = T;

    return j_matrix;
}

/**
 * @brief Convert to Eigen::Vector7d
 * @return Eigen::Vector7d{x, y, z, qw, qx, qy, qz}
 */
Eigen::Vector<double, 7> to_vector(const EtaQuat& eta) {
    return Eigen::Vector<double, 7>{eta.x,  eta.y,  eta.z, eta.qw,
                                    eta.qx, eta.qy, eta.qz};
}

/**
 * @brief Convert to Eta with Euler angles
 * @return Eta
 */
inline Eta as_eta_euler(const EtaQuat& eta_quat) {
    Eigen::Vector3d euler_angles = vortex::utils::math::quat_to_euler(
        vortex::utils::views::ori_quaternion(eta_quat).normalized());

    Eta eta{.x = eta.x,
            .y = eta.y,
            .z = eta.z,
            .roll = euler_angles(0),
            .pitch = euler_angles(1),
            .yaw = euler_angles(2)};
    return eta;
}

inline Nu nu_error(const Nu& nu1, const Nu& nu2) {
    Nu nu;
    nu.u = nu1.u - nu2.u;
    nu.v = nu1.v - nu2.v;
    nu.w = nu1.w - nu2.w;
    nu.p = nu1.p - nu2.p;
    nu.q = nu1.q - nu2.q;
    nu.r = nu1.r - nu2.r;
    return nu;
}

/**
 * @brief Convert to Eigen::Vector6d
 * @return Eigen::Vector6d{u, v, w, p, q, r}
 */
Eigen::Vector<double, 6> to_vector(const Nu& nu) {
    return Eigen::Vector<double, 6>{nu.u, nu.v, nu.w, nu.p, nu.q, nu.r};
}

}  // namespace vortex::utils::types

#endif  // VORTEX_UTILS__OPS_HPP_
