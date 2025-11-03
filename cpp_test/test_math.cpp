#include <gtest/gtest.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include "vortex/utils/math.hpp"

namespace vortex::utils::math {

// Test that the value does not change when already in the interval [-pi, pi]
TEST(ssa, test_ssa_0) {
    EXPECT_EQ(0, ssa(0));
}

// Test that 2 pi correctly maps to 0
TEST(ssa, test_ssa_2pi) {
    EXPECT_EQ(0, ssa(2 * M_PI));
}

// Test that values over pi gets mapped to the negative interval
TEST(ssa, test_ssa_3_5) {
    EXPECT_NEAR(-2.78, ssa(3.5), 0.01);
}

// Test that values under -pi gets mapped to the positive interval
TEST(ssa, test_ssa_minus_3_5) {
    EXPECT_NEAR(2.78, ssa(-3.5), 0.01);
}

TEST(ssa, test_ssa_minus_pi) {
    EXPECT_EQ(M_PI, ssa(-M_PI));
}

TEST(ssa, test_ssa_pi) {
    EXPECT_EQ(M_PI, ssa(M_PI));
}

TEST(ssa, test_ssa_vector) {
    Eigen::VectorXd angles(6);
    angles << 0.0, M_PI / 2, M_PI, 3 * M_PI, -4 * M_PI, -3 * M_PI / 2.0;

    Eigen::VectorXd expected(6);
    expected << 0.0, M_PI / 2, M_PI, M_PI, 0.0, M_PI / 2.0;

    Eigen::VectorXd result = ssa(angles);
    EXPECT_TRUE(result.isApprox(expected, 1e-12));

    std::vector<double> angle_vec{0.0,      M_PI / 2,  M_PI,
                                  3 * M_PI, -4 * M_PI, -3 * M_PI / 2.0};
    std::vector<double> expected_vec{0.0,  M_PI / 2, M_PI,
                                     M_PI, 0.0,      M_PI / 2.0};
    std::vector<double> result_vec = ssa(angle_vec);
    for (size_t i = 0; i < angle_vec.size(); ++i) {
        EXPECT_NEAR(expected_vec[i], result_vec[i], 1e-12);
    }

    std::array<double, 6> angle_array{0.0,      M_PI / 2,  M_PI,
                                      3 * M_PI, -4 * M_PI, -3 * M_PI / 2.0};
    std::array<double, 6> expected_array{0.0,  M_PI / 2, M_PI,
                                         M_PI, 0.0,      M_PI / 2.0};
    std::array<double, 6> result_array = ssa(angle_array);
    for (size_t i = 0; i < angle_array.size(); ++i) {
        EXPECT_NEAR(expected_array[i], result_array[i], 1e-12);
    }
}

// Test that the skew-symmetric matrix is correctly calculated
TEST(get_skew_symmetric_matrix, test_skew_symmetric) {
    Eigen::Vector3d vector(1, 2, 3);
    Eigen::Matrix3d expected;
    expected << 0, -3, 2, 3, 0, -1, -2, 1, 0;

    Eigen::Matrix3d result = get_skew_symmetric_matrix(vector);
    EXPECT_TRUE(result.isApprox(expected, 1e-12));
}

// Test that rotation matrix is correctly constructed
TEST(get_rotation_matrix, test_rotation_matrix) {
    double roll{1.0};
    double pitch{2.0};
    double yaw{3.0};
    Eigen::Matrix3d expected;
    expected << 0.41198225, -0.83373765, -0.36763046, -0.05872664, -0.42691762,
        0.90238159, -0.90929743, -0.35017549, -0.2248451;
    Eigen::Matrix3d result = get_rotation_matrix(roll, pitch, yaw);
    EXPECT_TRUE(result.isApprox(expected, 1e-8));
}

TEST(get_transformation_matrix_attitude, test_transformation_matrix_zeros) {
    Eigen::Matrix3d transformation_matrix{
        get_transformation_matrix_attitude(0.0, 0.0)};
    Eigen::Matrix3d expected{Eigen::Matrix3d::Identity()};
    EXPECT_TRUE(transformation_matrix.isApprox(expected, 1e-12));
}

TEST(get_transformation_matrix_attitude_quat,
     test_transformation_matrix_unit_quat) {
    Eigen::Quaterniond quat = Eigen::Quaterniond::Identity();
    Eigen::Matrix<double, 4, 3> transformation_matrix{
        get_transformation_matrix_attitude_quat(quat)};
    Eigen::Matrix<double, 4, 3> expected = Eigen::Matrix<double, 4, 3>::Zero();
    expected.bottomRightCorner<3, 3>() = 0.5 * Eigen::Matrix3d::Identity();
    EXPECT_TRUE(transformation_matrix.isApprox(expected, 1e-12));
}

TEST(eigen_vector3d_to_quaternion, zero_vector_returns_identity) {
    const Eigen::Vector3d v = Eigen::Vector3d::Zero();

    const Eigen::Quaterniond q = eigen_vector3d_to_quaternion(v);

    EXPECT_TRUE(q.isApprox(Eigen::Quaterniond::Identity(), 1e-12));
}

TEST(eigen_vector3d_to_quaternion, general_vector_matches_axis_angle_formula) {
    const Eigen::Vector3d v(0.3, -0.4, 0.5);
    const double angle = v.norm();
    const Eigen::Vector3d axis = v / angle;

    const double half = 0.5 * angle;
    const double c = cos(half);
    const double s = sin(half);
    const Eigen::Quaterniond expected(c, axis.x() * s, axis.y() * s,
                                      axis.z() * s);

    const Eigen::Quaterniond q = eigen_vector3d_to_quaternion(v);

    EXPECT_TRUE(q.isApprox(expected, 1e-12));
}

// Test that the identity quaternion correctly maps to 0, 0, 0
TEST(quat_to_euler, test_quat_to_euler_1) {
    Eigen::Quaterniond q(1.0, 0.0, 0.0, 0.0);
    Eigen::Vector3d expected(0.0, 0.0, 0.0);
    Eigen::Vector3d result = quat_to_euler(q);
    EXPECT_TRUE(result.isApprox(expected, 1e-3));
}

// Test that only changing w and x in the quat only affects roll
TEST(quat_to_euler, test_quat_to_euler_2) {
    Eigen::Quaterniond q2(0.707, 0.707, 0.0, 0.0);
    Eigen::Vector3d expected2(1.57, 0.0, 0.0);
    Eigen::Vector3d result2 = quat_to_euler(q2);
    EXPECT_TRUE(result2.isApprox(expected2, 1e-3));
}

// Test that only changing w and z in the quat only affects yaw
TEST(quat_to_euler, test_quat_to_euler_3) {
    Eigen::Quaterniond q3(0.707, 0.0, 0.0, 0.707);
    Eigen::Vector3d expected3(0.0, 0.0, 1.57);
    Eigen::Vector3d result3 = quat_to_euler(q3);
    EXPECT_TRUE(result3.isApprox(expected3, 1e-3));
}

// Test that a quaternion is correctly converted to euler angles
TEST(quat_to_euler, test_quat_to_euler_4) {
    Eigen::Quaterniond q4(0.770, 0.4207, -0.4207, -0.229);
    Eigen::Vector3d expected4(1.237, -0.4729, -0.9179);
    Eigen::Vector3d result4 = quat_to_euler(q4);
    EXPECT_TRUE(result4.isApprox(expected4, 1e-3));
}

// Test that a quaternion with flipped signs is correctly converted to euler
// angles
TEST(quat_to_euler, test_quat_to_euler_5) {
    Eigen::Quaterniond q5(0.770, 0.4207, 0.4207, 0.229);
    Eigen::Vector3d expected5(1.237, 0.4729, 0.9179);
    Eigen::Vector3d result5 = quat_to_euler(q5);

    EXPECT_TRUE(result5.isApprox(expected5, 1e-3));
}

// Test that zero euler angles construct the correct quaternion
TEST(euler_to_quat, test_euler_to_quat_1) {
    double roll{};
    double pitch{};
    double yaw{};
    Eigen::Quaterniond q{euler_to_quat(roll, pitch, yaw)};
    Eigen::Vector4d result{q.x(), q.y(), q.z(), q.w()};
    Eigen::Vector4d expected{0.0, 0.0, 0.0, 1.0};

    EXPECT_TRUE(result.isApprox(expected, 1e-3));
}

// Test that non-zero roll constructs the correct quaternion
TEST(euler_to_quat, test_euler_to_quat_2) {
    double roll{1.0};
    double pitch{};
    double yaw{};
    Eigen::Quaterniond q{euler_to_quat(roll, pitch, yaw)};
    Eigen::Vector4d result{q.x(), q.y(), q.z(), q.w()};
    Eigen::Vector4d expected{0.479, 0.0, 0.0, 0.877};

    EXPECT_TRUE(result.isApprox(expected, 1e-3));
}

// Test that non-zero pitch constructs the correct quaternion
TEST(euler_to_quat, test_euler_to_quat_3) {
    double roll{};
    double pitch{1.0};
    double yaw{};
    Eigen::Quaterniond q{euler_to_quat(roll, pitch, yaw)};
    Eigen::Vector4d result{q.x(), q.y(), q.z(), q.w()};
    Eigen::Vector4d expected{0.0, 0.479, 0.0, 0.877};

    EXPECT_TRUE(result.isApprox(expected, 1e-3));
}

// Test that non-zero yaw constructs the correct quaternion
TEST(euler_to_quat, test_euler_to_quat_4) {
    double roll{};
    double pitch{};
    double yaw{1.0};
    Eigen::Quaterniond q{euler_to_quat(roll, pitch, yaw)};
    Eigen::Vector4d result{q.x(), q.y(), q.z(), q.w()};
    Eigen::Vector4d expected{0.0, 0.0, 0.479, 0.877};

    EXPECT_TRUE(result.isApprox(expected, 1e-3));
}

// Test that non-zero euler angles constructs the correct quaternion
TEST(euler_to_quat, test_euler_to_quat_5) {
    double roll{1.0};
    double pitch{1.0};
    double yaw{1.0};
    Eigen::Quaterniond q{euler_to_quat(roll, pitch, yaw)};
    Eigen::Vector4d result{q.x(), q.y(), q.z(), q.w()};
    Eigen::Vector4d expected{0.1675, 0.5709, 0.1675, 0.786};

    EXPECT_TRUE(result.isApprox(expected, 1e-3));
}

// Test that zero euler angles construct the correct quaternion
TEST(euler_to_quat_vec3, zero_angles_identity_quaternion) {
    const Eigen::Vector3d euler(0.0, 0.0, 0.0);  // roll, pitch, yaw
    const Eigen::Quaterniond q = euler_to_quat(euler);

    const Eigen::Vector4d result{q.x(), q.y(), q.z(), q.w()};
    const Eigen::Vector4d expected(0.0, 0.0, 0.0, 1.0);

    EXPECT_TRUE(result.isApprox(expected, 1e-3));
}

// Test that non-zero roll constructs the correct quaternion
TEST(euler_to_quat_vec3, roll_only) {
    const Eigen::Vector3d euler(1.0, 0.0, 0.0);  // roll, pitch, yaw
    const Eigen::Quaterniond q = euler_to_quat(euler);

    const Eigen::Vector4d result{q.x(), q.y(), q.z(), q.w()};
    const Eigen::Vector4d expected(0.479, 0.0, 0.0, 0.877);

    EXPECT_TRUE(result.isApprox(expected, 1e-3));
}

// Test that non-zero pitch constructs the correct quaternion
TEST(euler_to_quat_vec3, pitch_only) {
    const Eigen::Vector3d euler(0.0, 1.0, 0.0);
    const Eigen::Quaterniond q = euler_to_quat(euler);

    const Eigen::Vector4d result{q.x(), q.y(), q.z(), q.w()};
    const Eigen::Vector4d expected(0.0, 0.479, 0.0, 0.877);

    EXPECT_TRUE(result.isApprox(expected, 1e-3));
}

// Test that non-zero yaw constructs the correct quaternion
TEST(euler_to_quat_vec3, yaw_only) {
    const Eigen::Vector3d euler(0.0, 0.0, 1.0);
    const Eigen::Quaterniond q = euler_to_quat(euler);

    const Eigen::Vector4d result{q.x(), q.y(), q.z(), q.w()};
    const Eigen::Vector4d expected(0.0, 0.0, 0.479, 0.877);

    EXPECT_TRUE(result.isApprox(expected, 1e-3));
}

// Test that non-zero euler angles construct the correct quaternion
TEST(euler_to_quat_vec3, roll_pitch_yaw_all_nonzero) {
    const Eigen::Vector3d euler(1.0, 1.0, 1.0);
    const Eigen::Quaterniond q = euler_to_quat(euler);

    const Eigen::Vector4d result{q.x(), q.y(), q.z(), q.w()};
    const Eigen::Vector4d expected(0.1675, 0.5709, 0.1675, 0.786);

    EXPECT_TRUE(result.isApprox(expected, 1e-3));
}

// Test pseudo inverse, results from
// https://www.emathhelp.net/calculators/linear-algebra/pseudoinverse-calculator
TEST(pseudo_inverse, pseudo_inverse_of_square_matrix_same_as_inverse) {
    Eigen::MatrixXd matrix(2, 2);
    matrix << 4, 7, 2, 6;
    Eigen::MatrixXd expected = matrix.inverse();
    Eigen::MatrixXd result = pseudo_inverse(matrix);
    EXPECT_TRUE(expected.isApprox(result, 1e-10));
}

TEST(pseudo_inverse, pseudo_inverse_of_2_by_3_matrix) {
    Eigen::MatrixXd matrix(2, 3);
    matrix << 1, 2, 3, 4, 5, 6;
    Eigen::MatrixXd expected(3, 2);
    expected << -17.0 / 18.0, 4.0 / 9.0, -1.0 / 9.0, 1.0 / 9.0, 13.0 / 18.0,
        -2.0 / 9.0;
    Eigen::MatrixXd result = pseudo_inverse(matrix);
    EXPECT_TRUE(expected.isApprox(result, 1e-10));
}

TEST(pseudo_inverse, pseudo_inverse_of_3_by_2_matrix) {
    Eigen::MatrixXd matrix(3, 2);
    matrix << 1, 2, 3, 4, 5, 6;
    Eigen::MatrixXd expected(2, 3);
    expected << -4.0 / 3.0, -1.0 / 3.0, 2.0 / 3.0, 13.0 / 12.0, 1.0 / 3.0,
        -5.0 / 12.0;
    Eigen::MatrixXd result = pseudo_inverse(matrix);
    EXPECT_TRUE(expected.isApprox(result, 1e-10));
}

TEST(clamp_values, test_clamp_values) {
    Eigen::Vector<double, 5> values;
    values << -10.0, -1.0, 0.0, 1.0, 10.0;
    double min_val = -5.0;
    double max_val = 5.0;
    Eigen::Vector<double, 5> expected;
    expected << -5.0, -1.0, 0.0, 1.0, 5.0;
    Eigen::Vector<double, 5> result = clamp_values(values, min_val, max_val);
    EXPECT_TRUE(expected.isApprox(result, 1e-10));
}

TEST(anti_windup, test_anti_windup) {
    double dt = 0.1;
    Eigen::Vector<double, 3> error;
    error << 1.0, -2.0, 3.0;
    Eigen::Vector<double, 3> integral;
    integral << 0.5, -0.5, 0.5;
    double min_val = -0.5;
    double max_val = 0.7;
    Eigen::Vector<double, 3> expected;
    expected << 0.6, -0.5, 0.7;  // Last value clamped
    Eigen::Vector<double, 3> result =
        anti_windup(dt, error, integral, min_val, max_val);
    EXPECT_TRUE(expected.isApprox(result, 1e-10));
}

}  // namespace vortex::utils::math
