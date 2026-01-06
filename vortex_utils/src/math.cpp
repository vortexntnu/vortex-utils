#include "vortex/utils/math.hpp"

namespace vortex::utils::math {

double ssa(const double angle) {
    double angle_ssa{fmod(angle + M_PI, 2 * M_PI)};
    return angle_ssa <= 0 ? angle_ssa + M_PI : angle_ssa - M_PI;
}

Eigen::Matrix3d get_skew_symmetric_matrix(const Eigen::Vector3d& vector) {
    Eigen::Matrix3d skew_symmetric_matrix;
    skew_symmetric_matrix << 0, -vector.z(), vector.y(), vector.z(), 0,
        -vector.x(), -vector.y(), vector.x(), 0;
    return skew_symmetric_matrix;
}

Eigen::Matrix3d get_rotation_matrix(const double roll,
                                    const double pitch,
                                    const double yaw) {
    Eigen::Matrix3d rotation_matrix =
        (Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
         Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
         Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()))
            .toRotationMatrix();
    return rotation_matrix;
}

Eigen::Matrix<double, 4, 3> get_transformation_matrix_attitude_quat(
    const Eigen::Quaterniond& quat) {
    Eigen::Matrix<double, 4, 3> T_q = Eigen::Matrix<double, 4, 3>::Zero();
    double qw{quat.w()};
    double qx{quat.x()};
    double qy{quat.y()};
    double qz{quat.z()};

    T_q << -qx, -qy, -qz, qw, -qz, qy, qz, qw, -qx, -qy, qx, qw;

    T_q *= 0.5;
    return T_q;
}

Eigen::Quaterniond eigen_vector3d_to_quaternion(const Eigen::Vector3d& vector) {
    double angle{vector.norm()};
    if (angle < 1e-8) {
        return Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0);
    } else {
        Eigen::Vector3d axis = vector / angle;
        Eigen::Quaterniond quat =
            Eigen::Quaterniond(Eigen::AngleAxisd(angle, axis));
        return quat.normalized();
    }
}

Eigen::Matrix3d get_transformation_matrix_attitude(const double roll,
                                                   const double pitch) {
    double sin_r = sin(roll);
    double cos_r = cos(roll);
    double cos_p = cos(pitch);
    if (cos_p == 0) {
        throw std::runtime_error("Singular pitch");
    }
    double tan_p = tan(pitch);

    Eigen::Matrix3d transformation_matrix;
    transformation_matrix(0, 0) = 1;
    transformation_matrix(0, 1) = sin_r * tan_p;
    transformation_matrix(0, 2) = cos_r * tan_p;
    transformation_matrix(1, 0) = 0;
    transformation_matrix(1, 1) = cos_r;
    transformation_matrix(1, 2) = -sin_r;
    transformation_matrix(2, 0) = 0;
    transformation_matrix(2, 1) = sin_r / cos_p;
    transformation_matrix(2, 2) = cos_r / cos_p;
    return transformation_matrix;
}

Eigen::Vector3d quat_to_euler(const Eigen::Quaterniond& q) {
    Eigen::Matrix3d rotation_matrix = q.toRotationMatrix();
    if (std::fabs(rotation_matrix(2, 0)) > 1.0) {
        throw std::runtime_error("Singular value in pitch");
    }
    double roll{atan2(rotation_matrix(2, 1), rotation_matrix(2, 2))};
    double pitch{-asin(rotation_matrix(2, 0))};
    double yaw{atan2(rotation_matrix(1, 0), rotation_matrix(0, 0))};
    return {roll, pitch, yaw};
}

Eigen::Quaterniond euler_to_quat(const double roll,
                                 const double pitch,
                                 const double yaw) {
    const Eigen::AngleAxisd r_z(yaw, Eigen::Vector3d::UnitZ());
    const Eigen::AngleAxisd r_y(pitch, Eigen::Vector3d::UnitY());
    const Eigen::AngleAxisd r_x(roll, Eigen::Vector3d::UnitX());
    Eigen::Quaterniond q = r_z * r_y * r_x;
    return q.normalized();
}

Eigen::Quaterniond euler_to_quat(const Eigen::Vector3d& euler) {
    Eigen::Quaterniond q;
    q = Eigen::AngleAxisd(euler.z(), Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(euler.y(), Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(euler.x(), Eigen::Vector3d::UnitX());
    return q.normalized();
}

Eigen::MatrixXd pseudo_inverse(const Eigen::MatrixXd& matrix) {
    if (matrix.rows() >= matrix.cols()) {
        return (matrix.transpose() * matrix).ldlt().solve(matrix.transpose());
    } else {
        return matrix.transpose() * (matrix * matrix.transpose())
                                        .ldlt()
                                        .solve(Eigen::MatrixXd::Identity(
                                            matrix.rows(), matrix.rows()));
    }
}

Eigen::VectorXd clamp_values(const Eigen::VectorXd& values,
                             const double min_val,
                             const double max_val) {
    return values.cwiseMax(min_val).cwiseMin(max_val);
}

Eigen::VectorXd anti_windup(const double dt,
                            const Eigen::VectorXd& error,
                            const Eigen::VectorXd& integral,
                            const double min_val,
                            const double max_val) {
    Eigen::VectorXd integral_anti_windup = integral + (error * dt);

    integral_anti_windup = clamp_values(integral_anti_windup, min_val, max_val);
    return integral_anti_windup;
}

Eigen::Quaterniond average_quaternions(
    const std::vector<Eigen::Quaterniond>& quaternions) {
    if (quaternions.empty()) {
        throw std::invalid_argument(
            "average_quaternions: input vector must not be empty");
    }

    Eigen::Matrix4d scatter_matrix = Eigen::Matrix4d::Zero();
    std::ranges::for_each(quaternions, [&](const auto& q) {
        scatter_matrix +=
            q.normalized().coeffs() * q.normalized().coeffs().transpose();
    });

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix4d> eigensolver(scatter_matrix);

    if (eigensolver.info() != Eigen::Success) {
        throw std::runtime_error(
            "average_quaternions: eigen decomposition failed after bias");
    }

    const auto& eigenvalues = eigensolver.eigenvalues();
    constexpr double eps = 1e-12;

    if (std::abs(eigenvalues(3) - eigenvalues(2)) < eps) {
        throw std::runtime_error(
            "average_quaternions_weighted: average orientation is not unique");
    }

    const Eigen::Vector4d eigenvector = eigensolver.eigenvectors().col(3);

    Eigen::Quaterniond avg_q;
    avg_q.x() = eigenvector(0);
    avg_q.y() = eigenvector(1);
    avg_q.z() = eigenvector(2);
    avg_q.w() = eigenvector(3);

    if (avg_q.w() < 0.0) {
        avg_q.coeffs() *= -1.0;
    }

    return avg_q.normalized();
}

Eigen::Quaterniond enu_ned_rotation(const Eigen::Quaterniond& quat) {
    const Eigen::Matrix3d rotation_matrix_enu_to_ned = [] {
        Eigen::Matrix3d rotmat;
        rotmat.col(0) = Eigen::Vector3d(0, 1, 0);
        rotmat.col(1) = Eigen::Vector3d(1, 0, 0);
        rotmat.col(2) = Eigen::Vector3d(0, 0, -1);
        return rotmat;
    }();

    Eigen::Quaterniond q_out =
        Eigen::Quaterniond(rotation_matrix_enu_to_ned) * quat.normalized();

    return q_out.normalized();
}

}  // namespace vortex::utils::math
