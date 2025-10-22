#ifndef VORTEX_UTILS_TYPES_HPP
#define VORTEX_UTILS_TYPES_HPP

#include <eigen3/Eigen/Dense>
#include "math.hpp"

namespace Eigen {
typedef Eigen::Matrix<double, 6, 6> Matrix6d;
typedef Eigen::Matrix<double, 6, 1> Vector6d;
}  // namespace Eigen

namespace vortex::utils::types {

// @brief Struct to represent the state vector eta,
// containing the position and orientation.
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

    Eigen::Vector6d to_vector() const {
        return Eigen::Vector6d{x, y, z, roll, pitch, yaw};
    }

    // @brief Make the rotation matrix according to eq. 2.31 in Fossen, 2021
    Eigen::Matrix3d as_rotation_matrix() const {
        Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
        Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());

        Eigen::Quaterniond q = yawAngle * pitchAngle * rollAngle;
        Eigen::Matrix3d rotation_matrix = q.toRotationMatrix();

        return rotation_matrix;
    }

    // @brief Make the transformation matrix according to eq. 2.41 in Fossen,
    // 2021
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

    Eigen::Matrix6d as_j_matrix() const {
        Eigen::Matrix3d rotation_matrix = as_rotation_matrix();
        Eigen::Matrix3d transformation_matrix = as_transformation_matrix();

        Eigen::Matrix6d j_matrix = Eigen::Matrix6d::Zero();
        j_matrix.block<3, 3>(0, 0) = rotation_matrix;
        j_matrix.block<3, 3>(3, 3) = transformation_matrix;

        return j_matrix;
    }
};

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

    Eigen::Vector6d to_vector() const {
        return Eigen::Vector6d{u, v, w, p, q, r};
    }
};

// @brief Struct to represent a 3D pose with position and orientation (quaternion).
struct Pose {
    Eigen::Vector3d position{Eigen::Vector3d::Zero()};
    Eigen::Quaterniond orientation{Eigen::Quaterniond::Identity()};

    Pose() = default;

    Pose(const Eigen::Vector3d& pos, const Eigen::Quaterniond& quat)
        : position(pos), orientation(quat.normalized()) {}

    Pose(double x, double y, double z, double roll, double pitch, double yaw)
        : position(x, y, z) {
        orientation = vortex::utils::math::euler_to_quat(roll, pitch, yaw);
    }
};


}  // namespace vortex::utils::types

#endif  // VORTEX_UTILS_TYPES_HPP
