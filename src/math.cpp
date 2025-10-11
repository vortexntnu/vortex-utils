#include "vortex_utils/math.hpp"

namespace vortex::utils::math {

double ssa(const double angle) {
    double angle_ssa { fmod(angle + M_PI, 2 * M_PI) };
    return angle_ssa < 0 ? angle_ssa + M_PI : angle_ssa - M_PI;
}

Eigen::Matrix3d get_skew_symmetric_matrix(const Eigen::Vector3d& vector) {
    Eigen::Matrix3d skew_symmetric_matrix;
    skew_symmetric_matrix << 0, -vector.z(), vector.y(), vector.z(), 0,
        -vector.x(), -vector.y(), vector.x(), 0;
    return skew_symmetric_matrix;
}

Eigen::Matrix3d get_rotation_matrix(const double roll, const double pitch, const double yaw) {
    Eigen::Matrix3d rotation_matrix =
      (Eigen::AngleAxisd(yaw,   Eigen::Vector3d::UnitZ()) *
       Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
       Eigen::AngleAxisd(roll,  Eigen::Vector3d::UnitX()))
      .toRotationMatrix();
    return rotation_matrix;
}

Eigen::Vector3d quat_to_euler(const Eigen::Quaterniond& q) {
    Eigen::Matrix3d rotation_matrix = q.toRotationMatrix();
    if (std::fabs(rotation_matrix(2, 0)) > 1.0) {
        throw std::runtime_error("Singular value in pitch");
    }
    double roll { atan2(rotation_matrix(2, 1), rotation_matrix(2, 2))};
    double pitch { -asin(rotation_matrix(2, 0)) };
    double yaw { atan2(rotation_matrix(1, 0), rotation_matrix(0, 0)) };
    return {roll, pitch, yaw};
}

Eigen::Quaterniond euler_to_quat(const double roll, const double pitch, const double yaw) {
    const Eigen::AngleAxisd r_z(yaw, Eigen::Vector3d::UnitZ());
    const Eigen::AngleAxisd r_y(pitch, Eigen::Vector3d::UnitY());
    const Eigen::AngleAxisd r_x(roll, Eigen::Vector3d::UnitX());
    Eigen::Quaterniond q = r_z * r_y * r_x;
    return q.normalized();
}

}  // namespace vortex::utils::math