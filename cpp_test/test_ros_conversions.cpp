#include <gtest/gtest.h>
#include <string>

#include <eigen3/Eigen/Geometry>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

#include "vortex/utils/math.hpp"
#include "vortex/utils/ros_conversions.hpp"
#include "vortex/utils/types.hpp"

struct HasEulerPose {
    double x = 0, y = 0, z = 0;
    double roll = 0, pitch = 0, yaw = 0;
};

struct HasEulerPoseExtra {
    double x = 0, y = 0, z = 0;
    double roll = 0, pitch = 0, yaw = 0;
    double extra = 1.0;
};

struct HasQuatPose {
    double x = 1, y = 2, z = 3;
    double qw = 1, qx = 0, qy = 0, qz = 0;
};

struct MissingYaw {
    double x = 0, y = 0, z = 0;
    double roll = 0, pitch = 0;
};

struct WrongEulerType {
    double x = 0, y = 0, z = 0;
    std::string roll;
    double pitch = 0, yaw = 0;
};

template <typename T>
concept AcceptsToPoseMsg =
    requires(const T& t) { vortex::utils::ros_conversions::to_pose_msg(t); };

static_assert(AcceptsToPoseMsg<HasEulerPose>);
static_assert(AcceptsToPoseMsg<HasEulerPoseExtra>);
static_assert(AcceptsToPoseMsg<HasQuatPose>);

static_assert(!AcceptsToPoseMsg<MissingYaw>);
static_assert(!AcceptsToPoseMsg<WrongEulerType>);

static_assert(!AcceptsToPoseMsg<Eigen::Vector3d>);
static_assert(!AcceptsToPoseMsg<Eigen::Matrix<double, 6, 1>>);
static_assert(!AcceptsToPoseMsg<Eigen::Matrix<double, 7, 1>>);

TEST(to_pose_msg, euler_zero) {
    HasEulerPose p;

    auto pose = vortex::utils::ros_conversions::to_pose_msg(p);

    EXPECT_NEAR(pose.position.x, 0.0, 1e-6);
    EXPECT_NEAR(pose.position.y, 0.0, 1e-6);
    EXPECT_NEAR(pose.position.z, 0.0, 1e-6);

    EXPECT_NEAR(pose.orientation.w, 1.0, 1e-6);
    EXPECT_NEAR(pose.orientation.x, 0.0, 1e-6);
    EXPECT_NEAR(pose.orientation.y, 0.0, 1e-6);
    EXPECT_NEAR(pose.orientation.z, 0.0, 1e-6);
}

TEST(to_pose_msg, euler_nonzero) {
    HasEulerPose p{1, 2, 3, 0.1, 0.2, 0.3};

    auto pose = vortex::utils::ros_conversions::to_pose_msg(p);

    Eigen::Quaterniond q = vortex::utils::math::euler_to_quat(0.1, 0.2, 0.3);

    EXPECT_NEAR(pose.position.x, 1.0, 1e-6);
    EXPECT_NEAR(pose.position.y, 2.0, 1e-6);
    EXPECT_NEAR(pose.position.z, 3.0, 1e-6);

    EXPECT_NEAR(pose.orientation.w, q.w(), 1e-6);
    EXPECT_NEAR(pose.orientation.x, q.x(), 1e-6);
    EXPECT_NEAR(pose.orientation.y, q.y(), 1e-6);
    EXPECT_NEAR(pose.orientation.z, q.z(), 1e-6);
}

TEST(to_pose_msg, quaternion_pose) {
    HasQuatPose p;

    auto pose = vortex::utils::ros_conversions::to_pose_msg(p);

    EXPECT_NEAR(pose.position.x, 1.0, 1e-6);
    EXPECT_NEAR(pose.position.y, 2.0, 1e-6);
    EXPECT_NEAR(pose.position.z, 3.0, 1e-6);

    EXPECT_NEAR(pose.orientation.w, 1.0, 1e-6);
    EXPECT_NEAR(pose.orientation.x, 0.0, 1e-6);
    EXPECT_NEAR(pose.orientation.y, 0.0, 1e-6);
    EXPECT_NEAR(pose.orientation.z, 0.0, 1e-6);
}

TEST(to_pose_msgs, vector_of_euler) {
    std::vector<HasEulerPose> poses(3);

    auto out = vortex::utils::ros_conversions::to_pose_msgs(poses);

    ASSERT_EQ(out.size(), 3);
}

TEST(ros_to_pose_vec, pose) {
    geometry_msgs::msg::Pose p;
    p.position.x = 1;
    p.position.y = 2;
    p.position.z = 3;
    p.orientation.w = 1;
    p.orientation.x = 0;
    p.orientation.y = 0;
    p.orientation.z = 0;

    auto v = vortex::utils::ros_conversions::ros_to_pose_vec(p);

    ASSERT_EQ(v.size(), 1);
    EXPECT_NEAR(v[0].x, 1.0, 1e-6);
    EXPECT_NEAR(v[0].y, 2.0, 1e-6);
    EXPECT_NEAR(v[0].z, 3.0, 1e-6);

    EXPECT_NEAR(v[0].qw, 1.0, 1e-6);
    EXPECT_NEAR(v[0].qz, 0.0, 1e-6);
    EXPECT_NEAR(v[0].qz, 0.0, 1e-6);
    EXPECT_NEAR(v[0].qz, 0.0, 1e-6);
}

TEST(ros_to_pose_vec, pose_stamped) {
    geometry_msgs::msg::PoseStamped p;
    p.pose.position.x = 1;
    p.pose.position.y = 2;
    p.pose.position.z = 3;
    p.pose.orientation.w = 1;
    p.pose.orientation.x = 0;
    p.pose.orientation.y = 0;
    p.pose.orientation.z = 0;

    auto v = vortex::utils::ros_conversions::ros_to_pose_vec(p);

    ASSERT_EQ(v.size(), 1);
    EXPECT_NEAR(v[0].x, 1.0, 1e-6);
    EXPECT_NEAR(v[0].y, 2.0, 1e-6);
    EXPECT_NEAR(v[0].z, 3.0, 1e-6);

    EXPECT_NEAR(v[0].qw, 1.0, 1e-6);
    EXPECT_NEAR(v[0].qz, 0.0, 1e-6);
    EXPECT_NEAR(v[0].qz, 0.0, 1e-6);
    EXPECT_NEAR(v[0].qz, 0.0, 1e-6);
}

TEST(ros_to_pose_vec, pose_with_covariance) {
    geometry_msgs::msg::PoseWithCovariance p;
    p.pose.position.x = 1;
    p.pose.position.y = 2;
    p.pose.position.z = 3;
    p.pose.orientation.w = 1;
    p.pose.orientation.x = 0;
    p.pose.orientation.y = 0;
    p.pose.orientation.z = 0;

    auto v = vortex::utils::ros_conversions::ros_to_pose_vec(p);

    ASSERT_EQ(v.size(), 1);
    EXPECT_NEAR(v[0].x, 1.0, 1e-6);
    EXPECT_NEAR(v[0].y, 2.0, 1e-6);
    EXPECT_NEAR(v[0].z, 3.0, 1e-6);

    EXPECT_NEAR(v[0].qw, 1.0, 1e-6);
    EXPECT_NEAR(v[0].qz, 0.0, 1e-6);
    EXPECT_NEAR(v[0].qz, 0.0, 1e-6);
    EXPECT_NEAR(v[0].qz, 0.0, 1e-6);
}

TEST(ros_to_pose_vec, pose_with_covariance_stamped) {
    geometry_msgs::msg::PoseWithCovarianceStamped p;
    p.pose.pose.position.x = 1;
    p.pose.pose.position.y = 2;
    p.pose.pose.position.z = 3;
    p.pose.pose.orientation.w = 1;
    p.pose.pose.orientation.x = 0;
    p.pose.pose.orientation.y = 0;
    p.pose.pose.orientation.z = 0;

    auto v = vortex::utils::ros_conversions::ros_to_pose_vec(p);

    ASSERT_EQ(v.size(), 1);
    EXPECT_NEAR(v[0].x, 1.0, 1e-6);
    EXPECT_NEAR(v[0].y, 2.0, 1e-6);
    EXPECT_NEAR(v[0].z, 3.0, 1e-6);

    EXPECT_NEAR(v[0].qw, 1.0, 1e-6);
    EXPECT_NEAR(v[0].qz, 0.0, 1e-6);
    EXPECT_NEAR(v[0].qz, 0.0, 1e-6);
    EXPECT_NEAR(v[0].qz, 0.0, 1e-6);
}

TEST(ros_to_pose_vec, pose_array) {
    geometry_msgs::msg::PoseArray arr;
    arr.poses.resize(2);

    arr.poses[0].position.x = 1;
    arr.poses[1].position.x = 2;

    auto v = vortex::utils::ros_conversions::ros_to_pose_vec(arr);

    ASSERT_EQ(v.size(), 2);
    EXPECT_NEAR(v[0].x, 1.0, 1e-6);
    EXPECT_NEAR(v[1].x, 2.0, 1e-6);
}
