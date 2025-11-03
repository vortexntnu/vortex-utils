#include <gtest/gtest.h>

#include "vortex/utils/math.hpp"
#include "vortex/utils/types.hpp"

class TypesTests : public ::testing::Test {
   public:
    TypesTests() = default;
    void SetUp() override {}
};

TEST_F(TypesTests, test_eta) {
    vortex::utils::types::Eta eta;
    // Test correct zero initialization
    EXPECT_EQ(eta.x, 0.0);
    EXPECT_EQ(eta.y, 0.0);
    EXPECT_EQ(eta.z, 0.0);
    EXPECT_EQ(eta.roll, 0.0);
    EXPECT_EQ(eta.pitch, 0.0);
    EXPECT_EQ(eta.yaw, 0.0);

    // Test rotation and transformation matrix
    eta.roll = 1.0;
    eta.pitch = 0.5;
    eta.yaw = 1.7;
    Eigen::Matrix3d expected_rm{
        vortex::utils::math::get_rotation_matrix(1.0, 0.5, 1.7)};
    Eigen::Matrix3d result_rm{eta.as_rotation_matrix()};
    EXPECT_TRUE(result_rm.isApprox(expected_rm, 1e-12));

    Eigen::Matrix3d expected_tm{
        vortex::utils::math::get_transformation_matrix_attitude(1.0, 0.5)};
    Eigen::Matrix3d result_tm{eta.as_transformation_matrix()};
    EXPECT_TRUE(result_tm.isApprox(expected_tm, 1e-12));

    // Test to_vector
    eta.x = 5.0;
    eta.y = -4.0;
    eta.z = 2.1;
    Eigen::Vector<double, 6> result_v{eta.to_vector()};
    Eigen::Vector<double, 6> expected_v{5.0, -4.0, 2.1, 1.0, 0.5, 1.7};
    EXPECT_TRUE(result_v.isApprox(expected_v, 1e-12));

    // Test operator-
    vortex::utils::types::Eta other{1.0, 2.0, 3.0, 0.1, 0.2, 0.3};
    vortex::utils::types::Eta diff{eta - other};
    EXPECT_NEAR(diff.x, 4.0, 1e-12);
    EXPECT_NEAR(diff.y, -6.0, 1e-12);
    EXPECT_NEAR(diff.z, -0.9, 1e-12);
    EXPECT_NEAR(diff.roll, 0.9, 1e-12);
    EXPECT_NEAR(diff.pitch, 0.3, 1e-12);
    EXPECT_NEAR(diff.yaw, 1.4, 1e-12);
}

TEST_F(TypesTests, test_eta_quat) {
    vortex::utils::types::EtaQuat eta_quat;
    // Test correct zero initialization
    EXPECT_EQ(eta_quat.x, 0.0);
    EXPECT_EQ(eta_quat.y, 0.0);
    EXPECT_EQ(eta_quat.z, 0.0);
    EXPECT_EQ(eta_quat.qw, 1.0);
    EXPECT_EQ(eta_quat.qx, 0.0);
    EXPECT_EQ(eta_quat.qy, 0.0);
    EXPECT_EQ(eta_quat.qz, 0.0);

    // Test to_vector
    eta_quat.x = 5.0;
    eta_quat.y = -4.0;
    eta_quat.z = 2.1;
    eta_quat.qw = 1.0;
    eta_quat.qx = 0.5;
    eta_quat.qy = -0.5;
    eta_quat.qz = 0.25;
    Eigen::Vector<double, 7> result_v{eta_quat.to_vector()};
    Eigen::Vector<double, 7> expected_v{5.0, -4.0, 2.1, 1.0, 0.5, -0.5, 0.25};
    EXPECT_TRUE(result_v.isApprox(expected_v, 1e-12));

    // Test operator-
    vortex::utils::types::EtaQuat other{1.0, 2.0, 3.0, 0.1, 0.2, 0.3, 0.4};
    vortex::utils::types::EtaQuat diff{eta_quat - other};
    auto pos = diff.pos_vector();
    EXPECT_TRUE(pos.isApprox(Eigen::Vector3d(4.0, -6.0, -0.9), 1e-12));
    auto q = diff.ori_quaternion();
    EXPECT_TRUE(q.isApprox(
        Eigen::Quaterniond(0.21908902300206642, -0.6207522318391883,
                           -0.7302967433402215, -0.18257418583505536),
        1e-12));
}

TEST_F(TypesTests, test_nu) {
    vortex::utils::types::Nu nu;
    // Test correct zero initialization
    EXPECT_EQ(nu.u, 0.0);
    EXPECT_EQ(nu.v, 0.0);
    EXPECT_EQ(nu.w, 0.0);
    EXPECT_EQ(nu.p, 0.0);
    EXPECT_EQ(nu.q, 0.0);
    EXPECT_EQ(nu.r, 0.0);

    // Test to_vector
    nu.u = 5.0;
    nu.v = -4.0;
    nu.w = 2.1;
    nu.p = 1.0;
    nu.q = 0.5;
    nu.r = 1.7;
    Eigen::Vector<double, 6> result_v{nu.to_vector()};
    Eigen::Vector<double, 6> expected_v{5.0, -4.0, 2.1, 1.0, 0.5, 1.7};
    EXPECT_TRUE(result_v.isApprox(expected_v, 1e-12));

    // Test operator-
    vortex::utils::types::Nu other{1.0, 2.0, 3.0, 0.1, 0.2, 0.3};
    vortex::utils::types::Nu diff{nu - other};
    EXPECT_NEAR(diff.u, 4.0, 1e-12);
    EXPECT_NEAR(diff.v, -6.0, 1e-12);
    EXPECT_NEAR(diff.w, -0.9, 1e-12);
    EXPECT_NEAR(diff.p, 0.9, 1e-12);
    EXPECT_NEAR(diff.q, 0.3, 1e-12);
    EXPECT_NEAR(diff.r, 1.4, 1e-12);
}
