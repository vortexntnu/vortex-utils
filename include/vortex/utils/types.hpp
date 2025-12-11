#ifndef VORTEX_UTILS_TYPES_HPP
#define VORTEX_UTILS_TYPES_HPP

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include "math.hpp"

namespace vortex::utils::types {

/**
 * @brief 6x1 column vector of doubles
*/
typedef Eigen::Matrix<double, 6, 1> Vector6d;

/**
 * @brief Struct to represent the state vector eta according to eq. 2.3 in
 * Fossen, 2021, containing the position and orientation.
 */
struct Eta;

/**
 * @brief Struct to represent the state vector eta according to eq. 2.3 in
 * Fossen, 2021, containing the position and orientation as quaternion.
 */
struct EtaQuat;

struct Eta {
    double x{};
    double y{};
    double z{};
    double roll{};
    double pitch{};
    double yaw{};

    Eta operator-(const Eta& other) const {
        Eta eta;
        eta.x = x - other.x;
        eta.y = y - other.y;
        eta.z = z - other.z;
        eta.roll = roll - other.roll;
        eta.pitch = pitch - other.pitch;
        eta.yaw = yaw - other.yaw;
        return eta;
    }

    /**
     * @brief Get the position vector (x, y, z).
     * @return Eigen::Vector3d{x, y, z}
     */
    Eigen::Vector3d pos_vector() const { return Eigen::Vector3d{x, y, z}; }

    /**
     * @brief Get the orientation vector (roll, pitch, yaw).
     * @return Eigen::Vector3d{roll, pitch, yaw}
     */
    Eigen::Vector3d ori_vector() const {
        return Eigen::Vector3d{roll, pitch, yaw};
    }

    /**
     * @brief Convert to Eigen::Vector6d
     * @return Eigen::Vector6d{x, y, z, roll, pitch, yaw}
     */
    Eigen::Vector<double, 6> to_vector() const {
        return Eigen::Vector<double, 6>{x, y, z, roll, pitch, yaw};
    }

    /**
     * @brief Apply smallest signed angle to roll, pitch, and yaw.
     */
    void apply_ssa() {
        roll = vortex::utils::math::ssa(roll);
        pitch = vortex::utils::math::ssa(pitch);
        yaw = vortex::utils::math::ssa(yaw);
    }

    /**
     * @brief Make the rotation matrix corresponding to eq. 2.31 in Fossen, 2021
     * @return Eigen::Matrix3d rotation matrix
     */
    Eigen::Matrix3d as_rotation_matrix() const {
        return vortex::utils::math::get_rotation_matrix(roll, pitch, yaw);
    }
    /**
     * @brief Make the transformation matrix according to eq. 2.41 in Fossen,
     * 2021
     * @return Eigen::Matrix3d transformation matrix
     */
    Eigen::Matrix3d as_transformation_matrix() const {
        double cphi = cos(roll);
        double sphi = sin(roll);
        double ctheta = cos(pitch);
        double stheta = sin(pitch);

        if (ctheta == 0) {
            throw std::runtime_error(
                "Division by zero in transformation matrix.");
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
    Eigen::Matrix<double, 6, 6> as_j_matrix() const {
        Eigen::Matrix3d rotation_matrix = as_rotation_matrix();
        Eigen::Matrix3d transformation_matrix = as_transformation_matrix();

        Eigen::Matrix<double, 6, 6> j_matrix =
            Eigen::Matrix<double, 6, 6>::Zero();
        j_matrix.topLeftCorner<3, 3>() = rotation_matrix;
        j_matrix.bottomRightCorner<3, 3>() = transformation_matrix;

        return j_matrix;
    }
    /**
     * @brief Convert to EtaQuat
     * @return EtaQuat
     */
    EtaQuat as_eta_quat() const;
};

struct EtaQuat {
    double x{};
    double y{};
    double z{};
    double qw{1.0};
    double qx{};
    double qy{};
    double qz{};

    /**
     * @brief Get the position vector (x, y, z).
     * @return Eigen::Vector3d{x, y, z}
     */
    Eigen::Vector3d pos_vector() const { return Eigen::Vector3d{x, y, z}; }

    /**
     * @brief Get the orientation as Eigen::Quaterniond.
     * @return Eigen::Quaterniond
     */
    Eigen::Quaterniond ori_quaternion() const {
        Eigen::Quaterniond quat;
        quat.w() = qw;
        quat.x() = qx;
        quat.y() = qy;
        quat.z() = qz;
        return quat.normalized();
    }

    /**
     * @brief Convert to Eigen::Vector7d
     * @return Eigen::Vector7d{x, y, z, qw, qx, qy, qz}
     */
    Eigen::Vector<double, 7> to_vector() const {
        return Eigen::Vector<double, 7>{x, y, z, qw, qx, qy, qz};
    }

    EtaQuat operator-(const EtaQuat& other) const {
        EtaQuat eta;
        eta.x = x - other.x;
        eta.y = y - other.y;
        eta.z = z - other.z;
        Eigen::Quaterniond q1 = ori_quaternion();
        Eigen::Quaterniond q2 = other.ori_quaternion();
        Eigen::Quaterniond q_error = q2.conjugate() * q1;
        q_error.normalize();
        eta.qw = q_error.w();
        eta.qx = q_error.x();
        eta.qy = q_error.y();
        eta.qz = q_error.z();
        return eta;
    }

    /**
     * @brief Make the rotation matrix corresponding to eq. 2.31 in Fossen, 2021
     * @return Eigen::Matrix3d rotation matrix
     */
    Eigen::Matrix3d as_rotation_matrix() const {
        return ori_quaternion().toRotationMatrix();
    }

    /**
     * @brief Make the transformation matrix according to eq. 2.78 in Fossen,
     * 2021
     * @return Eigen::Matrix<double, 4, 3> transformation matrix
     */
    Eigen::Matrix<double, 4, 3> as_transformation_matrix() const {
        return vortex::utils::math::get_transformation_matrix_attitude_quat(
            ori_quaternion());
    }

    /**
     * @brief Make the J matrix according to eq. 2.83 in Fossen, 2021
     * @return Eigen::Matrix<double, 7, 6> J matrix
     */
    Eigen::Matrix<double, 7, 6> as_j_matrix() const {
        Eigen::Matrix3d R = as_rotation_matrix();
        Eigen::Matrix<double, 4, 3> T = as_transformation_matrix();

        Eigen::Matrix<double, 7, 6> j_matrix =
            Eigen::Matrix<double, 7, 6>::Zero();
        j_matrix.topLeftCorner<3, 3>() = R;
        j_matrix.bottomRightCorner<4, 3>() = T;

        return j_matrix;
    }
    /**
     * @brief Convert to Eta with Euler angles
     * @return Eta
     */
    Eta as_eta_euler() const;
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

    Nu operator-(const Nu& other) const {
        Nu nu;
        nu.u = u - other.u;
        nu.v = v - other.v;
        nu.w = w - other.w;
        nu.p = p - other.p;
        nu.q = q - other.q;
        nu.r = r - other.r;
        return nu;
    }

    /**
     * @brief Convert to Eigen::Vector6d
     * @return Eigen::Vector6d{u, v, w, p, q, r}
     */
    Eigen::Vector<double, 6> to_vector() const {
        return Eigen::Vector<double, 6>{u, v, w, p, q, r};
    }
};

inline EtaQuat Eta::as_eta_quat() const {
    Eigen::Quaterniond quat =
        vortex::utils::math::euler_to_quat(roll, pitch, yaw);

    EtaQuat eta_quat{.x = x,
                     .y = y,
                     .z = z,
                     .qw = quat.w(),
                     .qx = quat.x(),
                     .qy = quat.y(),
                     .qz = quat.z()};
    return eta_quat;
}

inline Eta EtaQuat::as_eta_euler() const {
    Eigen::Vector3d euler_angles =
        vortex::utils::math::quat_to_euler(ori_quaternion());

    Eta eta{.x = x,
            .y = y,
            .z = z,
            .roll = euler_angles(0),
            .pitch = euler_angles(1),
            .yaw = euler_angles(2)};
    return eta;
}

}  // namespace vortex::utils::types

#endif  // VORTEX_UTILS_TYPES_HPP
