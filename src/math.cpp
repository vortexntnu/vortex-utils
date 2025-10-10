#include "vortex_utils/math.hpp"

namespace vortex::utils::math {

double ssa(const double angle) {
    double angle_ssa { fmod(angle + M_PI, 2 * M_PI) };
    return angle_ssa < 0 ? angle_ssa + M_PI : angle_ssa - M_PI;
}

Eigen::Matrix3d skew_symmetric(const Eigen::Vector3d& vector) {
    Eigen::Matrix3d skew_symmetric_matrix;
    skew_symmetric_matrix << 0, -vector.z(), vector.y(), vector.z(), 0,
        -vector.x(), -vector.y(), vector.x(), 0;
    return skew_symmetric_matrix;
}

Eigen::Vector3d quat_to_euler(const Eigen::Quaterniond& q) {
    Eigen::Vector3d euler_angles = q.toRotationMatrix().eulerAngles(0, 1, 2);
    return euler_angles;
}

}  // namespace vortex::utils::math