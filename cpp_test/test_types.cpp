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
    double diff_rm{
        vortex::utils::math::matrix_norm_diff(expected_rm, result_rm)};
    EXPECT_NEAR(diff_rm, 0.0, 1e-12);

    Eigen::Matrix3d expected_tm{
        vortex::utils::math::get_transformation_matrix_attitude(1.0, 0.5)};
    Eigen::Matrix3d result_tm{eta.as_transformation_matrix()};
    double diff_tm{
        vortex::utils::math::matrix_norm_diff(expected_tm, result_tm)};
    EXPECT_NEAR(diff_tm, 0.0, 1e-12);

    // Test to_vector
    eta.x = 5.0;
    eta.y = -4.0;
    eta.z = 2.1;
    Eigen::Vector6d result_v{eta.to_vector()};
    Eigen::Vector6d expected_v{5.0, -4.0, 2.1, 1.0, 0.5, 1.7};
    for (int i = 0; i < 6; ++i) {
        EXPECT_NEAR(expected_v[i], result_v[i], 1e-12);
    }

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
    Eigen::Vector6d result_v{nu.to_vector()};
    Eigen::Vector6d expected_v{5.0, -4.0, 2.1, 1.0, 0.5, 1.7};
    for (int i = 0; i < 6; ++i) {
        EXPECT_NEAR(expected_v[i], result_v[i], 1e-12);
    }

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
