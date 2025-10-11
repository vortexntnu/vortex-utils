#include <gtest/gtest.h>
#include "vortex_utils/math.hpp"

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

// @brief Helper to calculate error between two matrices
inline double matrix_norm_diff(Eigen::Matrix3d m1, Eigen::Matrix3d m2) {
    return (m1 - m2).norm();
}

// Test that the skew-symmetric matrix is correctly calculated
TEST(get_skew_symmetric_matrix, test_skew_symmetric) {
    Eigen::Vector3d vector(1, 2, 3);
    Eigen::Matrix3d expected;
    expected << 0, -3, 2, 3, 0, -1, -2, 1, 0;

    Eigen::Matrix3d result = get_skew_symmetric_matrix(vector);
    EXPECT_EQ(expected, result);
}

// Test that rotation matrix is correctly constructed
TEST(get_rotation_matrix, test_rotation_matrix) {
    double roll { 1.0 };
    double pitch { 2.0 };
    double yaw { 3.0 };
    Eigen::Matrix3d expected;
    expected << 0.41198225, -0.83373765, -0.36763046, 
               -0.05872664, -0.42691762,  0.90238159,
               -0.90929743, -0.35017549, -0.2248451;
    Eigen::Matrix3d result = get_rotation_matrix(roll, pitch, yaw);
    EXPECT_NEAR(0.0, matrix_norm_diff(expected, result), 0.01);
}

// Test that the identity quaternion correctly maps to 0, 0, 0
TEST(quat_to_euler, test_quat_to_euler_1) {
    Eigen::Quaterniond q(1.0, 0.0, 0.0, 0.0);
    Eigen::Vector3d expected(0.0, 0.0, 0.0);
    Eigen::Vector3d result = quat_to_euler(q);
    for (int i = 0; i < 3; ++i) {
        EXPECT_NEAR(expected[i], result[i], 0.01);
    }
}

// Test that only changing w and x in the quat only affects roll
TEST(quat_to_euler, test_quat_to_euler_2) {
    Eigen::Quaterniond q2(0.707, 0.707, 0.0, 0.0);
    Eigen::Vector3d expected2(1.57, 0.0, 0.0);
    Eigen::Vector3d result2 = quat_to_euler(q2);
    for (int i = 0; i < 3; ++i) {
        EXPECT_NEAR(expected2[i], result2[i], 0.01);
    }
}

// Test that only changing w and z in the quat only affects yaw
TEST(quat_to_euler, test_quat_to_euler_3) {
    Eigen::Quaterniond q4(0.707, 0.0, 0.0, 0.707);
    Eigen::Vector3d expected4(0.0, 0.0, 1.57);
    Eigen::Vector3d result4 = quat_to_euler(q4);
    for (int i = 0; i < 3; ++i) {
        EXPECT_NEAR(expected4[i], result4[i], 0.01);
    }
}

// Test that a quaternion is correctly converted to euler angles
TEST(quat_to_euler, test_quat_to_euler_4) {
    Eigen::Quaterniond q5(0.770, 0.4207, -0.4207, -0.229);
    Eigen::Vector3d expected5(1.237, -0.4729, -0.9179);
    Eigen::Vector3d result5 = quat_to_euler(q5);
    for (int i = 0; i < 3; ++i) {
        EXPECT_NEAR(expected5[i], result5[i], 0.01);
    }
}

// Test that a quaternion with flipped signs is correctly convverted to euler angles
TEST(quat_to_euler, test_quat_to_euler_5) {
    Eigen::Quaterniond q5(0.770, 0.4207, 0.4207, 0.229);
    Eigen::Vector3d expected5(1.237, 0.4729, 0.9179);
    Eigen::Vector3d result5 = quat_to_euler(q5);
    for (int i = 0; i < 3; ++i) {
        EXPECT_NEAR(expected5[i], result5[i], 0.01);
    }
}

// Test that zero euler angles construct the correct quaternion
TEST(euler_to_quat, test_euler_to_quat_1) {
    double roll {};
    double pitch {};
    double yaw {};
    Eigen::Quaterniond q { euler_to_quat(roll, pitch, yaw) };
    Eigen::Vector4d result { q.x(), q.y(), q.z(), q.w() };
    Eigen::Vector4d expected { 0.0, 0.0, 0.0, 1.0 };
    for (int i = 0; i < 3; ++i) {
        EXPECT_NEAR(expected[i], result[i], 0.01);
    }
}

// Test that non-zero roll constructs the correct quaternion
TEST(euler_to_quat, test_euler_to_quat_2) {
    double roll { 1.0 };
    double pitch {};
    double yaw {};
    Eigen::Quaterniond q { euler_to_quat(roll, pitch, yaw) };
    Eigen::Vector4d result { q.x(), q.y(), q.z(), q.w() };
    Eigen::Vector4d expected { 0.479, 0.0, 0.0, 0.877 };
    for (int i = 0; i < 3; ++i) {
        EXPECT_NEAR(expected[i], result[i], 0.01);
    }
}

// Test that non-zero pitch constructs the correct quaternion
TEST(euler_to_quat, test_euler_to_quat_3) {
    double roll {};
    double pitch { 1.0 };
    double yaw {};
    Eigen::Quaterniond q { euler_to_quat(roll, pitch, yaw) };
    Eigen::Vector4d result { q.x(), q.y(), q.z(), q.w() };
    Eigen::Vector4d expected { 0.0, 0.479, 0.0, 0.877 };
    for (int i = 0; i < 3; ++i) {
        EXPECT_NEAR(expected[i], result[i], 0.01);
    }
}

// Test that non-zero yaw constructs the correct quaternion
TEST(euler_to_quat, test_euler_to_quat_4) {
    double roll {};
    double pitch {};
    double yaw { 1.0 };
    Eigen::Quaterniond q { euler_to_quat(roll, pitch, yaw) };
    Eigen::Vector4d result { q.x(), q.y(), q.z(), q.w() };
    Eigen::Vector4d expected { 0.0, 0.0, 0.479, 0.877 };
    for (int i = 0; i < 3; ++i) {
        EXPECT_NEAR(expected[i], result[i], 0.01);
    }
}

// Test that non-zero euler angles constructs the correct quaternion
TEST(euler_to_quat, test_euler_to_quat_5) {
    double roll { 1.0 };
    double pitch { 1.0 };
    double yaw { 1.0 };
    Eigen::Quaterniond q { euler_to_quat(roll, pitch, yaw) };
    Eigen::Vector4d result { q.x(), q.y(), q.z(), q.w() };
    Eigen::Vector4d expected { 0.1675, 0.5709, 0.1675, 0.786 };
    for (int i = 0; i < 3; ++i) {
        EXPECT_NEAR(expected[i], result[i], 0.01);
    }
}

}  // namespace vortex::utils::math