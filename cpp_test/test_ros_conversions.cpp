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

// ================================================================
//      Compile-Time Tests for ros_to_eigen6d Type Acceptance
// ================================================================

template <typename T>
concept AcceptsRosToEigen =
    requires(const T& t) { vortex::utils::ros_conversions::ros_to_eigen6d(t); };

static_assert(AcceptsRosToEigen<geometry_msgs::msg::Pose>,
              "Pose must be accepted");

static_assert(AcceptsRosToEigen<geometry_msgs::msg::PoseStamped>,
              "PoseStamped must be accepted");

static_assert(AcceptsRosToEigen<geometry_msgs::msg::PoseWithCovarianceStamped>,
              "PoseWithCovarianceStamped must be accepted");

static_assert(AcceptsRosToEigen<geometry_msgs::msg::PoseArray>,
              "PoseArray must be accepted");

struct RandomType {
    double x = 0;
};

static_assert(!AcceptsRosToEigen<int>, "int must NOT be accepted");

static_assert(!AcceptsRosToEigen<double>, "double must NOT be accepted");

static_assert(!AcceptsRosToEigen<Eigen::Vector3d>,
              "Eigen::Vector3d must NOT be accepted");

static_assert(!AcceptsRosToEigen<RandomType>,
              "RandomType must NOT be accepted");

static_assert(!AcceptsRosToEigen<std::shared_ptr<geometry_msgs::msg::Pose>>,
              "shared_ptr<Pose> must NOT be accepted");

static_assert(!AcceptsRosToEigen<geometry_msgs::msg::Pose*>,
              "raw pointer Pose* must NOT be accepted");

// ================================================================
//                   Runtime Tests: ros_to_eigen6d
// ================================================================
TEST(ros_to_eigen6d, pose_basic) {
    geometry_msgs::msg::Pose p;
    p.position.x = 1.0;
    p.position.y = 2.0;
    p.position.z = 3.0;

    Eigen::Quaterniond q = vortex::utils::math::euler_to_quat(0.1, 0.2, 0.3);
    p.orientation.x = q.x();
    p.orientation.y = q.y();
    p.orientation.z = q.z();
    p.orientation.w = q.w();

    auto v = vortex::utils::ros_conversions::ros_to_eigen6d(p);

    EXPECT_NEAR(v(0), 1.0, 1e-6);
    EXPECT_NEAR(v(1), 2.0, 1e-6);
    EXPECT_NEAR(v(2), 3.0, 1e-6);

    Eigen::Vector3d euler = vortex::utils::math::quat_to_euler(q);

    EXPECT_NEAR(v(3), euler(0), 1e-6);
    EXPECT_NEAR(v(4), euler(1), 1e-6);
    EXPECT_NEAR(v(5), euler(2), 1e-6);
}

TEST(ros_to_eigen6d, pose_stamped) {
    geometry_msgs::msg::PoseStamped ps;
    ps.pose.position.x = -1.0;
    ps.pose.position.y = 5.0;
    ps.pose.position.z = 10.0;

    Eigen::Quaterniond q = vortex::utils::math::euler_to_quat(0.3, 0.2, 0.1);
    ps.pose.orientation.x = q.x();
    ps.pose.orientation.y = q.y();
    ps.pose.orientation.z = q.z();
    ps.pose.orientation.w = q.w();

    auto v = vortex::utils::ros_conversions::ros_to_eigen6d(ps);

    EXPECT_NEAR(v(0), -1.0, 1e-6);
    EXPECT_NEAR(v(1), 5.0, 1e-6);
    EXPECT_NEAR(v(2), 10.0, 1e-6);

    Eigen::Vector3d euler = vortex::utils::math::quat_to_euler(q);
    EXPECT_NEAR(v(3), euler(0), 1e-6);
    EXPECT_NEAR(v(4), euler(1), 1e-6);
    EXPECT_NEAR(v(5), euler(2), 1e-6);
}

TEST(ros_to_eigen6d, pose_with_covariance_stamped) {
    geometry_msgs::msg::PoseWithCovarianceStamped pc;
    pc.pose.pose.position.x = 7.0;
    pc.pose.pose.position.y = 8.0;
    pc.pose.pose.position.z = 9.0;

    Eigen::Quaterniond q = vortex::utils::math::euler_to_quat(-0.1, 0.5, -0.3);
    pc.pose.pose.orientation.x = q.x();
    pc.pose.pose.orientation.y = q.y();
    pc.pose.pose.orientation.z = q.z();
    pc.pose.pose.orientation.w = q.w();

    auto v = vortex::utils::ros_conversions::ros_to_eigen6d(pc);

    EXPECT_NEAR(v(0), 7.0, 1e-6);
    EXPECT_NEAR(v(1), 8.0, 1e-6);
    EXPECT_NEAR(v(2), 9.0, 1e-6);

    Eigen::Vector3d euler = vortex::utils::math::quat_to_euler(q);
    EXPECT_NEAR(v(3), euler(0), 1e-6);
    EXPECT_NEAR(v(4), euler(1), 1e-6);
    EXPECT_NEAR(v(5), euler(2), 1e-6);
}

TEST(ros_to_eigen6d, pose_array_single) {
    geometry_msgs::msg::PoseArray arr;

    geometry_msgs::msg::Pose p;
    p.position.x = 4.0;
    p.position.y = 3.0;
    p.position.z = 2.0;

    Eigen::Quaterniond q = vortex::utils::math::euler_to_quat(0.4, -0.2, 1.0);
    p.orientation.x = q.x();
    p.orientation.y = q.y();
    p.orientation.z = q.z();
    p.orientation.w = q.w();

    arr.poses.push_back(p);

    auto X = vortex::utils::ros_conversions::ros_to_eigen6d(arr);

    ASSERT_EQ(X.cols(), 1);
    EXPECT_NEAR(X(0, 0), 4.0, 1e-6);
    EXPECT_NEAR(X(1, 0), 3.0, 1e-6);
    EXPECT_NEAR(X(2, 0), 2.0, 1e-6);

    Eigen::Vector3d euler = vortex::utils::math::quat_to_euler(q);
    EXPECT_NEAR(X(3, 0), euler(0), 1e-6);
    EXPECT_NEAR(X(4, 0), euler(1), 1e-6);
    EXPECT_NEAR(X(5, 0), euler(2), 1e-6);
}

TEST(ros_to_eigen6d, pose_array_multiple) {
    geometry_msgs::msg::PoseArray arr;

    for (int i = 0; i < 3; i++) {
        geometry_msgs::msg::Pose p;
        p.position.x = i;
        p.position.y = i + 1;
        p.position.z = i + 2;

        Eigen::Quaterniond q =
            vortex::utils::math::euler_to_quat(0.1 * i, 0.2 * i, 0.3 * i);
        p.orientation.x = q.x();
        p.orientation.y = q.y();
        p.orientation.z = q.z();
        p.orientation.w = q.w();

        arr.poses.push_back(p);
    }

    auto X = vortex::utils::ros_conversions::ros_to_eigen6d(arr);

    ASSERT_EQ(X.cols(), 3);

    for (int i = 0; i < 3; i++) {
        EXPECT_NEAR(X(0, i), i, 1e-6);
        EXPECT_NEAR(X(1, i), i + 1, 1e-6);
        EXPECT_NEAR(X(2, i), i + 2, 1e-6);

        Eigen::Quaterniond q =
            vortex::utils::math::euler_to_quat(0.1 * i, 0.2 * i, 0.3 * i);
        Eigen::Vector3d euler = vortex::utils::math::quat_to_euler(q);

        EXPECT_NEAR(X(3, i), euler(0), 1e-6);
        EXPECT_NEAR(X(4, i), euler(1), 1e-6);
        EXPECT_NEAR(X(5, i), euler(2), 1e-6);
    }
}
