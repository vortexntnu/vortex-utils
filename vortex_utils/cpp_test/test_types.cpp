#include <gtest/gtest.h>

#include "vortex/utils/math.hpp"
#include "vortex/utils/types.hpp"

class TypesTests : public ::testing::Test {
   public:
    TypesTests() = default;
    void SetUp() override {}
};

TEST_F(TypesTests, test_pose) {
    vortex::utils::types::PoseEuler pose;
    // Test correct zero initialization
    EXPECT_EQ(pose.x, 0.0);
    EXPECT_EQ(pose.y, 0.0);
    EXPECT_EQ(pose.z, 0.0);
    EXPECT_EQ(pose.roll, 0.0);
    EXPECT_EQ(pose.pitch, 0.0);
    EXPECT_EQ(pose.yaw, 0.0);

    // Test rotation and transformation matrix
    pose.roll = 1.0;
    pose.pitch = 0.5;
    pose.yaw = 1.7;
    Eigen::Matrix3d expected_rm{
        vortex::utils::math::get_rotation_matrix(1.0, 0.5, 1.7)};
    Eigen::Matrix3d result_rm{pose.as_rotation_matrix()};
    EXPECT_TRUE(result_rm.isApprox(expected_rm, 1e-12));

    Eigen::Matrix3d expected_tm{
        vortex::utils::math::get_transformation_matrix_attitude(1.0, 0.5)};
    Eigen::Matrix3d result_tm{pose.as_transformation_matrix()};
    EXPECT_TRUE(result_tm.isApprox(expected_tm, 1e-12));

    // Test to_vector
    pose.x = 5.0;
    pose.y = -4.0;
    pose.z = 2.1;
    Eigen::Vector<double, 6> result_v{pose.to_vector()};
    Eigen::Vector<double, 6> expected_v{5.0, -4.0, 2.1, 1.0, 0.5, 1.7};
    EXPECT_TRUE(result_v.isApprox(expected_v, 1e-12));

    // Test operator-
    vortex::utils::types::PoseEuler other{1.0, 2.0, 3.0, 0.1, 0.2, 0.3};
    vortex::utils::types::PoseEuler diff{pose - other};
    EXPECT_NEAR(diff.x, 4.0, 1e-12);
    EXPECT_NEAR(diff.y, -6.0, 1e-12);
    EXPECT_NEAR(diff.z, -0.9, 1e-12);
    EXPECT_NEAR(diff.roll, 0.9, 1e-12);
    EXPECT_NEAR(diff.pitch, 0.3, 1e-12);
    EXPECT_NEAR(diff.yaw, 1.4, 1e-12);
}

TEST_F(TypesTests, test_pose_quat) {
    vortex::utils::types::Pose pose_quat;
    // Test correct zero initialization
    EXPECT_EQ(pose_quat.x, 0.0);
    EXPECT_EQ(pose_quat.y, 0.0);
    EXPECT_EQ(pose_quat.z, 0.0);
    EXPECT_EQ(pose_quat.qw, 1.0);
    EXPECT_EQ(pose_quat.qx, 0.0);
    EXPECT_EQ(pose_quat.qy, 0.0);
    EXPECT_EQ(pose_quat.qz, 0.0);

    // Test to_vector
    pose_quat.x = 5.0;
    pose_quat.y = -4.0;
    pose_quat.z = 2.1;
    pose_quat.qw = 1.0;
    pose_quat.qx = 0.5;
    pose_quat.qy = -0.5;
    pose_quat.qz = 0.25;
    Eigen::Vector<double, 7> result_v{pose_quat.to_vector()};
    Eigen::Vector<double, 7> expected_v{5.0, -4.0, 2.1, 1.0, 0.5, -0.5, 0.25};
    EXPECT_TRUE(result_v.isApprox(expected_v, 1e-12));

    // Test operator-
    vortex::utils::types::Pose other{1.0, 2.0, 3.0, 0.1, 0.2, 0.3, 0.4};
    vortex::utils::types::Pose diff{pose_quat - other};
    auto pos = diff.pos_vector();
    EXPECT_TRUE(pos.isApprox(Eigen::Vector3d(4.0, -6.0, -0.9), 1e-12));
    auto q = diff.ori_quaternion();
    EXPECT_TRUE(q.isApprox(
        Eigen::Quaterniond(0.21908902300206642, -0.6207522318391883,
                           -0.7302967433402215, -0.18257418583505536),
        1e-12));
}

TEST_F(TypesTests, test_pose_from_eigen) {
    using vortex::utils::types::Pose;

    Eigen::Vector3d pos(1.0, -2.0, 3.5);

    // Deliberately NOT normalized
    Eigen::Quaterniond ori(2.0, -1.0, 0.5, 0.25);

    Pose pose = Pose::from_eigen(pos, ori);

    // --- Position mapping ---
    EXPECT_DOUBLE_EQ(pose.x, pos.x());
    EXPECT_DOUBLE_EQ(pose.y, pos.y());
    EXPECT_DOUBLE_EQ(pose.z, pos.z());

    // --- Orientation normalization ---
    Eigen::Quaterniond expected_q = ori.normalized();
    Eigen::Quaterniond result_q(pose.qw, pose.qx, pose.qy, pose.qz);

    EXPECT_TRUE(result_q.isApprox(expected_q, 1e-12));

    // --- Quaternion must be unit length ---
    EXPECT_NEAR(result_q.norm(), 1.0, 1e-12);
}

TEST_F(TypesTests, test_twist) {
    vortex::utils::types::Twist twist;
    // Test correct zero initialization
    EXPECT_EQ(twist.u, 0.0);
    EXPECT_EQ(twist.v, 0.0);
    EXPECT_EQ(twist.w, 0.0);
    EXPECT_EQ(twist.p, 0.0);
    EXPECT_EQ(twist.q, 0.0);
    EXPECT_EQ(twist.r, 0.0);

    // Test to_vector
    twist.u = 5.0;
    twist.v = -4.0;
    twist.w = 2.1;
    twist.p = 1.0;
    twist.q = 0.5;
    twist.r = 1.7;
    Eigen::Vector<double, 6> result_v{twist.to_vector()};
    Eigen::Vector<double, 6> expected_v{5.0, -4.0, 2.1, 1.0, 0.5, 1.7};
    EXPECT_TRUE(result_v.isApprox(expected_v, 1e-12));

    // Test operator-
    vortex::utils::types::Twist other{1.0, 2.0, 3.0, 0.1, 0.2, 0.3};
    vortex::utils::types::Twist diff{twist - other};
    EXPECT_NEAR(diff.u, 4.0, 1e-12);
    EXPECT_NEAR(diff.v, -6.0, 1e-12);
    EXPECT_NEAR(diff.w, -0.9, 1e-12);
    EXPECT_NEAR(diff.p, 0.9, 1e-12);
    EXPECT_NEAR(diff.q, 0.3, 1e-12);
    EXPECT_NEAR(diff.r, 1.4, 1e-12);
}

TEST_F(TypesTests, test_default_focals_throw) {
    vortex::utils::types::CameraIntrinsics intr;

    EXPECT_THROW(intr.K(), std::runtime_error);
}

TEST_F(TypesTests, test_negative_focals_throw) {
    vortex::utils::types::CameraIntrinsics intr{
        .fx = -1.0, .fy = -1.0, .cx = 100, .cy = 100, .skew = 0.0};

    EXPECT_THROW(intr.K(), std::runtime_error);
}

TEST_F(TypesTests, test_camera_intrinsics_K_no_skew) {
    vortex::utils::types::CameraIntrinsics intr{
        .fx = 300.0, .fy = 400.0, .cx = 150.0, .cy = 200.0, .skew = 0.0};

    Eigen::Matrix3d K = intr.K();

    Eigen::Matrix3d expected;
    expected << 300.0, 0.0, 150.0, 0.0, 400.0, 200.0, 0.0, 0.0, 1.0;

    EXPECT_TRUE(K.isApprox(expected, 1e-12));
}

TEST_F(TypesTests, test_camera_intrinsics_K_with_skew) {
    vortex::utils::types::CameraIntrinsics intr{
        .fx = 300.0, .fy = 400.0, .cx = 150.0, .cy = 200.0, .skew = 10.0};

    Eigen::Matrix3d K = intr.K();

    Eigen::Matrix3d expected;
    expected << 300.0, 10.0, 150.0, 0.0, 400.0, 200.0, 0.0, 0.0, 1.0;

    EXPECT_TRUE(K.isApprox(expected, 1e-12));
}

TEST_F(TypesTests, test_camera_intrinsics_K_inv_no_skew) {
    vortex::utils::types::CameraIntrinsics intr{
        .fx = 200.0, .fy = 400.0, .cx = 100.0, .cy = 50.0, .skew = 0.0};

    Eigen::Matrix3d K_inv = intr.K_inv();

    Eigen::Matrix3d expected;
    expected << 1.0 / 200.0, 0.0, -100.0 / 200.0, 0.0, 1.0 / 400.0,
        -50.0 / 400.0, 0.0, 0.0, 1.0;

    EXPECT_TRUE(K_inv.isApprox(expected, 1e-12));
}

TEST_F(TypesTests, test_camera_intrinsics_K_times_K_inv_identity) {
    vortex::utils::types::CameraIntrinsics intr{
        .fx = 200.0, .fy = 400.0, .cx = 100.0, .cy = 50.0, .skew = 0.0};

    Eigen::Matrix3d I = intr.K() * intr.K_inv();
    EXPECT_TRUE(I.isApprox(Eigen::Matrix3d::Identity(), 1e-12));
}

TEST_F(TypesTests, test_project) {
    vortex::utils::types::CameraIntrinsics intr{
        .fx = 300.0, .fy = 300.0, .cx = 150.0, .cy = 150.0, .skew = 0.0};

    Eigen::Vector3d point(10.0, 10.0, 10.0);

    Eigen::Vector2d pixel = intr.project_point(point);

    EXPECT_NEAR(pixel.x(), 450.0, 1e-12);
    EXPECT_NEAR(pixel.y(), 450.0, 1e-12);
}

TEST_F(TypesTests, test_project_with_skew) {
    vortex::utils::types::CameraIntrinsics intr{
        .fx = 300.0, .fy = 300.0, .cx = 100.0, .cy = 100.0, .skew = 50.0};

    Eigen::Vector3d point(2.0, 1.0, 1.0);

    Eigen::Vector2d pixel = intr.project_point(point);

    const double x_norm = 2.0;
    const double y_norm = 1.0;

    EXPECT_NEAR(pixel.x(), 300.0 * x_norm + 50.0 * y_norm + 100.0, 1e-12);
    EXPECT_NEAR(pixel.y(), 300.0 * y_norm + 100.0, 1e-12);
}

TEST_F(TypesTests, test_project_throws_on_nonpositive_z) {
    vortex::utils::types::CameraIntrinsics intr{
        .fx = 300.0, .fy = 300.0, .cx = 150.0, .cy = 150.0, .skew = 0.0};

    Eigen::Vector3d point(1.0, 1.0, 0.0);

    EXPECT_THROW(intr.project_point(point), std::runtime_error);
}
