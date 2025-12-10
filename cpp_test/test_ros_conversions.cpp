#include <gtest/gtest.h>
#include <string>
#include "vortex/utils/ros_conversions.hpp"
#include "vortex/utils/types.hpp"

// ---------- Compile-time concept tests ----------

// Valid type for EulerPoseLike
struct ValidEulerPose {
    double x, y, z;
    double roll, pitch, yaw;
};

// Valid type for EulerPoseLike with additional fields
struct ValidEulerPoseExtra {
    double x, y, z;
    double roll, pitch, yaw;
    double extra_field;  // additional field
};

// Invalid type for EulerPoseLike with quaternion fields
struct QuatPose {
    double x, y, z;
    double qx, qy, qz, qw;
};

// Invalid: missing yaw
struct MissingYaw {
    double x, y, z;
    double roll, pitch;
};

// Invalid: wrong type for roll
struct WrongType {
    double x, y, z;
    std::string roll;  // not convertible to double
    double pitch;
    double yaw;
};

static_assert(vortex::utils::ros_conversions::EulerPoseLike<ValidEulerPose>,
              "ValidEulerPose should satisfy EulerPoseLike");

static_assert(
    vortex::utils::ros_conversions::EulerPoseLike<ValidEulerPoseExtra>,
    "ValidEulerPoseExtra should satisfy EulerPoseLike");

static_assert(!vortex::utils::ros_conversions::EulerPoseLike<QuatPose>,
              "QuatPose should NOT satisfy EulerPoseLike");

static_assert(!vortex::utils::ros_conversions::EulerPoseLike<MissingYaw>,
              "MissingYaw should NOT satisfy EulerPoseLike");

static_assert(!vortex::utils::ros_conversions::EulerPoseLike<WrongType>,
              "WrongType should NOT satisfy EulerPoseLike");

// Ensure the function template accepts valid types
static_assert(
    std::is_same_v<
        decltype(vortex::utils::ros_conversions::euler_pose_to_pose_msg(
            std::declval<ValidEulerPose>())),
        geometry_msgs::msg::Pose>,
    "euler_pose_to_pose_msg should accept ValidEulerPose");

// Ensure the function template rejects invalid types
template <typename T>
concept FunctionAccepts = requires(T t) {
    vortex::utils::ros_conversions::euler_pose_to_pose_msg(t);
};

static_assert(FunctionAccepts<ValidEulerPose>,
              "Function should accept ValidEulerPose");

static_assert(FunctionAccepts<ValidEulerPoseExtra>,
              "Function should accept ValidEulerPoseExtra");

static_assert(!FunctionAccepts<QuatPose>,
              "Function should NOT accept QuatPose");

static_assert(!FunctionAccepts<MissingYaw>,
              "Function should NOT accept MissingYaw");

static_assert(!FunctionAccepts<WrongType>,
              "Function should NOT accept WrongType");

// ---------- Runtime tests ----------

TEST(euler_pose_to_pose_msg, test_default_eta_conversion) {
    vortex::utils::types::Eta eta;

    auto pose = vortex::utils::ros_conversions::euler_pose_to_pose_msg(eta);

    EXPECT_NEAR(pose.position.x, eta.x, 1e-3);
    EXPECT_NEAR(pose.position.y, eta.y, 1e-3);
    EXPECT_NEAR(pose.position.z, eta.z, 1e-3);

    // Expect identity quaternion for zero roll, pitch, yaw
    EXPECT_NEAR(pose.orientation.x, 0.0, 1e-3);
    EXPECT_NEAR(pose.orientation.y, 0.0, 1e-3);
    EXPECT_NEAR(pose.orientation.z, 0.0, 1e-3);
    EXPECT_NEAR(pose.orientation.w, 1.0, 1e-3);
}

TEST(euler_pose_to_pose_msg, test_nonzero_rotation) {
    struct Pose {
        double x = 1.0, y = 2.0, z = 3.0;
        double roll = 1.0;
        double pitch = 1.0;
        double yaw = 1.0;
    };

    Pose p;

    auto pose = vortex::utils::ros_conversions::euler_pose_to_pose_msg(p);

    EXPECT_NEAR(pose.position.x, 1.0, 1e-3);
    EXPECT_NEAR(pose.position.y, 2.0, 1e-3);
    EXPECT_NEAR(pose.position.z, 3.0, 1e-3);

    EXPECT_NEAR(pose.orientation.x, 0.1675, 1e-3);
    EXPECT_NEAR(pose.orientation.y, 0.5463, 1e-3);
    EXPECT_NEAR(pose.orientation.z, 0.1675, 1e-3);
    EXPECT_NEAR(pose.orientation.w, 0.786, 1e-3);
}
