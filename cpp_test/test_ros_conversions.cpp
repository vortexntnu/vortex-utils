#include <gtest/gtest.h>
#include <eigen3/Eigen/Dense>
#include <string>

#include "vortex/utils/ros_conversions.hpp"
#include "vortex/utils/types.hpp"

// ================================================================
//                  Compile-Time Concept Tests
// ================================================================

struct ValidEulerPose {
    double x = 0, y = 0, z = 0;
    double roll = 0, pitch = 0, yaw = 0;
};

struct ValidEulerPoseExtra {
    double x = 0, y = 0, z = 0;
    double roll = 0, pitch = 0, yaw = 0;
    double something_extra = 42.0;
};

struct ValidQuatPose {
    double x = 1, y = 2, z = 3;
    double qw = 1, qx = 0, qy = 0, qz = 0;
};

struct MissingYaw {
    double x = 0, y = 0, z = 0;
    double roll = 0, pitch = 0;
};

struct WrongType {
    double x = 0, y = 0, z = 0;
    std::string roll;  // not convertible to double
    double pitch = 0, yaw = 0;
};

struct MissingQuaternionField {
    double x = 0, y = 0, z = 0;
    double qx = 0, qy = 0, qz = 0;  // missing qw
};

// ================================================================
//                Concept: EulerPoseLike
// ================================================================
static_assert(vortex::utils::ros_conversions::EulerPoseLike<ValidEulerPose>,
              "ValidEulerPose should satisfy EulerPoseLike");

static_assert(
    vortex::utils::ros_conversions::EulerPoseLike<ValidEulerPoseExtra>,
    "ValidEulerPoseExtra should satisfy EulerPoseLike");

static_assert(!vortex::utils::ros_conversions::EulerPoseLike<MissingYaw>,
              "MissingYaw should NOT satisfy EulerPoseLike");

static_assert(!vortex::utils::ros_conversions::EulerPoseLike<WrongType>,
              "WrongType should NOT satisfy EulerPoseLike");

static_assert(
    !vortex::utils::ros_conversions::EulerPoseLike<ValidQuatPose>,
    "ValidQuatPose uses quaternion fields and must NOT satisfy EulerPoseLike");

// ================================================================
//                Concept: QuatPoseLike
// ================================================================
static_assert(vortex::utils::ros_conversions::QuatPoseLike<ValidQuatPose>,
              "ValidQuatPose should satisfy QuatPoseLike");

static_assert(
    !vortex::utils::ros_conversions::QuatPoseLike<MissingQuaternionField>,
    "MissingQuaternionField should NOT satisfy QuatPoseLike");

static_assert(!vortex::utils::ros_conversions::QuatPoseLike<ValidEulerPose>,
              "Euler pose must NOT satisfy QuatPoseLike");

// ================================================================
//                Concept: Eigen6dEuler
// ================================================================
static_assert(
    vortex::utils::ros_conversions::Eigen6dEuler<Eigen::Matrix<double, 6, 1>>,
    "Eigen::Vector6d should satisfy Eigen6dEuler");

static_assert(!vortex::utils::ros_conversions::Eigen6dEuler<Eigen::Vector3d>,
              "Eigen::Vector3d must NOT satisfy Eigen6dEuler");

// ================================================================
//                Concept: PoseLike (master)
// ================================================================
static_assert(vortex::utils::ros_conversions::PoseLike<ValidEulerPose>,
              "Euler pose should satisfy PoseLike");

static_assert(vortex::utils::ros_conversions::PoseLike<ValidQuatPose>,
              "Quat pose should satisfy PoseLike");

static_assert(
    vortex::utils::ros_conversions::PoseLike<Eigen::Matrix<double, 6, 1>>,
    "Eigen::Vector6d pose should satisfy PoseLike");

static_assert(!vortex::utils::ros_conversions::PoseLike<WrongType>,
              "WrongType must NOT satisfy PoseLike");

// ================================================================
//      Function Acceptance Using Concepts (Overload Resolution)
// ================================================================
template <typename T>
concept AcceptsPose =
    requires(T t) { vortex::utils::ros_conversions::pose_like_to_pose_msg(t); };

static_assert(AcceptsPose<ValidEulerPose>, "Euler pose should be accepted");

static_assert(AcceptsPose<ValidQuatPose>, "Quaternion pose should be accepted");

static_assert(AcceptsPose<Eigen::Matrix<double, 6, 1>>,
              "Eigen6d should be accepted");

static_assert(!AcceptsPose<MissingYaw>, "MissingYaw should NOT be accepted");

static_assert(!AcceptsPose<WrongType>, "WrongType should NOT be accepted");

// ================================================================
//                      Runtime Tests: Euler
// ================================================================
TEST(pose_like_to_pose_msg, euler_zero_eta) {
    vortex::utils::types::Eta eta;

    auto pose = vortex::utils::ros_conversions::pose_like_to_pose_msg(eta);

    EXPECT_NEAR(pose.position.x, 0.0, 1e-6);
    EXPECT_NEAR(pose.position.y, 0.0, 1e-6);
    EXPECT_NEAR(pose.position.z, 0.0, 1e-6);

    EXPECT_NEAR(pose.orientation.x, 0.0, 1e-6);
    EXPECT_NEAR(pose.orientation.y, 0.0, 1e-6);
    EXPECT_NEAR(pose.orientation.z, 0.0, 1e-6);
    EXPECT_NEAR(pose.orientation.w, 1.0, 1e-6);
}

TEST(pose_like_to_pose_msg, euler_nonzero_angles) {
    struct EP {
        double x = 1, y = 2, z = 3;
        double roll = 1, pitch = 1, yaw = 1;
    };
    EP p;

    auto pose = vortex::utils::ros_conversions::pose_like_to_pose_msg(p);

    EXPECT_NEAR(pose.position.x, 1, 1e-6);
    EXPECT_NEAR(pose.position.y, 2, 1e-6);
    EXPECT_NEAR(pose.position.z, 3, 1e-6);

    Eigen::Quaterniond expected = vortex::utils::math::euler_to_quat(1, 1, 1);

    EXPECT_NEAR(pose.orientation.x, expected.x(), 1e-6);
    EXPECT_NEAR(pose.orientation.y, expected.y(), 1e-6);
    EXPECT_NEAR(pose.orientation.z, expected.z(), 1e-6);
    EXPECT_NEAR(pose.orientation.w, expected.w(), 1e-6);
}

// ================================================================
//                Runtime Tests: Quaternion Pose
// ================================================================
TEST(pose_like_to_pose_msg, quat_pose_conversion) {
    ValidQuatPose qp;

    auto pose = vortex::utils::ros_conversions::pose_like_to_pose_msg(qp);

    EXPECT_NEAR(pose.position.x, 1.0, 1e-6);
    EXPECT_NEAR(pose.position.y, 2.0, 1e-6);
    EXPECT_NEAR(pose.position.z, 3.0, 1e-6);

    EXPECT_NEAR(pose.orientation.w, 1.0, 1e-6);
    EXPECT_NEAR(pose.orientation.x, 0.0, 1e-6);
    EXPECT_NEAR(pose.orientation.y, 0.0, 1e-6);
    EXPECT_NEAR(pose.orientation.z, 0.0, 1e-6);
}

// ================================================================
//                Runtime Tests: Eigen::Vector6d
// ================================================================
TEST(pose_like_to_pose_msg, eigen6d_conversion) {
    Eigen::Matrix<double, 6, 1> v;
    v << 1, 2, 3,       // xyz
        0.1, 0.2, 0.3;  // roll pitch yaw

    auto pose = vortex::utils::ros_conversions::pose_like_to_pose_msg(v);

    EXPECT_NEAR(pose.position.x, 1.0, 1e-6);
    EXPECT_NEAR(pose.position.y, 2.0, 1e-6);
    EXPECT_NEAR(pose.position.z, 3.0, 1e-6);

    Eigen::Quaterniond expected =
        vortex::utils::math::euler_to_quat(0.1, 0.2, 0.3);

    EXPECT_NEAR(pose.orientation.w, expected.w(), 1e-6);
    EXPECT_NEAR(pose.orientation.x, expected.x(), 1e-6);
    EXPECT_NEAR(pose.orientation.y, expected.y(), 1e-6);
    EXPECT_NEAR(pose.orientation.z, expected.z(), 1e-6);
}
