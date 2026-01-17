#ifndef VORTEX_UTILS_TYPES_HPP
#define VORTEX_UTILS_TYPES_HPP

#include <cmath>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <stdexcept>
#include "math.hpp"

namespace vortex::utils::types {

/**
 * @brief Struct to represent the pose vector according to eq. 2.3 in
 * Fossen, 2021, containing the position and orientation as euler angles.
 */
struct PoseEuler;

/**
 * @brief Struct to represent the pose vector according to eq. 2.3 in
 * Fossen, 2021, containing the position and orientation as quaternion.
 */
struct Pose;

/**
 * @brief Struct to represent the velocity vector according to eq. 2.5 in
 * Fossen, 2021, containing the linear and angular velocities.
 */
struct Twist;

/**
 * @brief Polar (Hesse normal) representation of a 2D line.
 *
 * @details
 * - rho is the signed distance from the origin to the line along the normal.
 * - theta is the angle (in radians) from the +x axis to the line's unit normal.
 */
struct Line2D;

/**
 * @brief Struct to represent a 2D line segment.
 */
struct LineSegment2D;

struct PoseEuler {
    double x{};
    double y{};
    double z{};
    double roll{};
    double pitch{};
    double yaw{};

    PoseEuler operator-(const PoseEuler& other) const {
        PoseEuler eta;
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
     * @brief Convert to Pose
     * @return Pose
     */
    Pose as_pose() const;
};

struct Pose {
    double x{};
    double y{};
    double z{};
    double qw{1.0};
    double qx{};
    double qy{};
    double qz{};

    /**
     * @brief Construct a Pose from eigen components.
     * @param pos Eigen::Vector3d position component
     * @param ori Eigen::Quaterniond orientation component
     * @return Pose with normalized quaternion
     */
    static Pose from_eigen(const Eigen::Vector3d& pos,
                           const Eigen::Quaterniond& ori) {
        const Eigen::Quaterniond q = ori.normalized();
        return Pose{.x = pos.x(),
                    .y = pos.y(),
                    .z = pos.z(),
                    .qw = q.w(),
                    .qx = q.x(),
                    .qy = q.y(),
                    .qz = q.z()};
    }

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
     * @brief Set the position from an Eigen::Vector3d.
     * @param pos Eigen::Vector3d
     */
    void set_pos(const Eigen::Vector3d& pos) {
        x = pos.x();
        y = pos.y();
        z = pos.z();
    }

    /**
     * @brief Set the orientation from an Eigen::Quaterniond.
     * @param ori Eigen::Quaterniond
     */
    void set_ori(const Eigen::Quaterniond& ori) {
        Eigen::Quaterniond q = ori.normalized();
        qw = q.w();
        qx = q.x();
        qy = q.y();
        qz = q.z();
    }

    /**
     * @brief Convert to Eigen::Vector7d
     * @return Eigen::Vector7d{x, y, z, qw, qx, qy, qz}
     */
    Eigen::Vector<double, 7> to_vector() const {
        return Eigen::Vector<double, 7>{x, y, z, qw, qx, qy, qz};
    }

    Pose operator-(const Pose& other) const {
        Pose eta;
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
     * @brief Convert to PoseEuler with Euler angles
     * @return PoseEuler
     */
    PoseEuler as_pose_euler() const;
};

/**
 * @brief Struct to represent the state vector nu according to eq. 2.5 in
 * Fossen, 2021, containing the linear and angular velocities.
 */
struct Twist {
    double u{};
    double v{};
    double w{};
    double p{};
    double q{};
    double r{};

    Twist operator-(const Twist& other) const {
        Twist nu;
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

inline Pose PoseEuler::as_pose() const {
    Eigen::Quaterniond quat =
        vortex::utils::math::euler_to_quat(roll, pitch, yaw);

    Pose eta_quat{.x = x,
                  .y = y,
                  .z = z,
                  .qw = quat.w(),
                  .qx = quat.x(),
                  .qy = quat.y(),
                  .qz = quat.z()};
    return eta_quat;
}

inline PoseEuler Pose::as_pose_euler() const {
    Eigen::Vector3d euler_angles =
        vortex::utils::math::quat_to_euler(ori_quaternion());

    PoseEuler eta{.x = x,
                  .y = y,
                  .z = z,
                  .roll = euler_angles(0),
                  .pitch = euler_angles(1),
                  .yaw = euler_angles(2)};
    return eta;
}

struct Point2D {
    double x{};
    double y{};
};

struct Line2D {
    double rho{};    ///< Distance from origin to the line in canonical form.
    double theta{};  ///< Angle (rad) from +x axis to the unit normal.
};

struct LineSegment2D {
    Point2D p0{};
    Point2D p1{};

    Line2D polar_parametrization() const {
        const double dx = p1.x - p0.x;
        const double dy = p1.y - p0.y;

        const double len = std::hypot(dx, dy);

        if (len <= std::numeric_limits<double>::epsilon()) {
            throw std::runtime_error("Invalid length for line segment");
        }

        const double nx = -dy / len;
        const double ny = dx / len;

        double rho = nx * p0.x + ny * p0.y;
        double theta = std::atan2(ny, nx);  // (-pi, pi]

        // Enforce positive rho
        if (rho < 0) {
            rho = -rho;
            theta += M_PI;
        }

        theta = std::fmod(theta, 2.0 * M_PI);

        if (theta < 0) {
            theta += 2.0 * M_PI;
        }

        return Line2D{.rho = rho, .theta = theta};
    }
};

}  // namespace vortex::utils::types

#endif  // VORTEX_UTILS_TYPES_HPP
