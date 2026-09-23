#include <gtest/gtest.h>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <random>
#include <vortex/utils/math.hpp>
#include <vortex/utils/waypoint_utils.hpp>

namespace vortex::utils::waypoints {

using vortex::utils::types::Pose;
using vortex::utils::types::WaypointMode;

// --- compute_waypoint_goal tests ---

TEST(ComputeWaypointGoal, FullPoseReturnsInputUnchanged) {
    Pose incoming{1.0, 2.0, 3.0, 1.0, 0.0, 0.0, 0.0};
    Pose current{};

    Pose result =
        compute_waypoint_goal(incoming, WaypointMode::FULL_POSE, current);

    EXPECT_DOUBLE_EQ(result.x, 1.0);
    EXPECT_DOUBLE_EQ(result.y, 2.0);
    EXPECT_DOUBLE_EQ(result.z, 3.0);
    EXPECT_DOUBLE_EQ(result.qw, 1.0);
    EXPECT_DOUBLE_EQ(result.qx, 0.0);
    EXPECT_DOUBLE_EQ(result.qy, 0.0);
    EXPECT_DOUBLE_EQ(result.qz, 0.0);
}

TEST(ComputeWaypointGoal, OnlyPositionKeepsOrientationFromState) {
    Pose incoming{1.0, 2.0, 3.0, 1.0, 0.0, 0.0, 0.0};

    // Current state with a non-identity orientation (90 deg about Z)
    Eigen::Quaterniond q(Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitZ()));
    Pose current = Pose::from_eigen(Eigen::Vector3d(10.0, 20.0, 30.0), q);

    Pose result =
        compute_waypoint_goal(incoming, WaypointMode::ONLY_POSITION, current);

    EXPECT_DOUBLE_EQ(result.x, 1.0);
    EXPECT_DOUBLE_EQ(result.y, 2.0);
    EXPECT_DOUBLE_EQ(result.z, 3.0);

    // Orientation should match current state
    Eigen::Quaterniond result_q = result.ori_quaternion();
    EXPECT_TRUE(result_q.isApprox(q.normalized(), 1e-12));
}

TEST(ComputeWaypointGoal, ForwardHeadingComputesYawFromDelta) {
    Pose incoming{1.0, 1.0, 0.0, 1.0, 0.0, 0.0, 0.0};
    Pose current{};

    Pose result =
        compute_waypoint_goal(incoming, WaypointMode::FORWARD_HEADING, current);

    EXPECT_DOUBLE_EQ(result.x, 1.0);
    EXPECT_DOUBLE_EQ(result.y, 1.0);

    // Expected yaw = atan2(1, 1) = pi/4
    double expected_yaw = std::atan2(1.0, 1.0);
    Eigen::Quaterniond expected_q(
        Eigen::AngleAxisd(expected_yaw, Eigen::Vector3d::UnitZ()));

    Eigen::Quaterniond result_q = result.ori_quaternion();
    EXPECT_TRUE(result_q.isApprox(expected_q.normalized(), 1e-12));
}

TEST(ComputeWaypointGoal, OnlyOrientationKeepsPositionFromState) {
    Eigen::Quaterniond q(Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitZ()));
    Pose incoming = Pose::from_eigen(Eigen::Vector3d(1.0, 2.0, 3.0), q);
    Pose current{10.0, 20.0, 30.0, 1.0, 0.0, 0.0, 0.0};

    Pose result = compute_waypoint_goal(
        incoming, WaypointMode::ONLY_ORIENTATION, current);

    EXPECT_DOUBLE_EQ(result.x, 10.0);
    EXPECT_DOUBLE_EQ(result.y, 20.0);
    EXPECT_DOUBLE_EQ(result.z, 30.0);

    Eigen::Quaterniond result_q = result.ori_quaternion();
    EXPECT_TRUE(result_q.isApprox(q.normalized(), 1e-12));
}

// --- has_converged tests ---

TEST(HasConverged, FullPoseBelowThreshold) {
    Pose measured{1.0, 2.0, 3.0, 1.0, 0.0, 0.0, 0.0};
    Pose reference{1.001, 2.0, 3.0, 1.0, 0.0, 0.0, 0.0};

    EXPECT_TRUE(
        has_converged(measured, reference, WaypointMode::FULL_POSE, 0.1));
}

TEST(HasConverged, FullPoseAboveThreshold) {
    Pose measured{};
    Pose reference{1.0, 1.0, 1.0, 1.0, 0.0, 0.0, 0.0};

    EXPECT_FALSE(
        has_converged(measured, reference, WaypointMode::FULL_POSE, 0.1));
}

TEST(HasConverged, OnlyPositionIgnoresOrientation) {
    Pose measured{1.0, 2.0, 3.0, 1.0, 0.0, 0.0, 0.0};

    // Same position, very different orientation
    Eigen::Quaterniond q(Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitZ()));
    Pose reference = Pose::from_eigen(Eigen::Vector3d(1.0, 2.0, 3.0), q);

    EXPECT_TRUE(
        has_converged(measured, reference, WaypointMode::ONLY_POSITION, 0.1));
}

TEST(HasConverged, OnlyOrientationIgnoresPosition) {
    Eigen::Quaterniond q(Eigen::AngleAxisd(0.1, Eigen::Vector3d::UnitZ()));
    Pose measured = Pose::from_eigen(Eigen::Vector3d(100.0, 200.0, 300.0), q);
    Pose reference = Pose::from_eigen(Eigen::Vector3d(0.0, 0.0, 0.0), q);

    EXPECT_TRUE(has_converged(measured, reference,
                              WaypointMode::ONLY_ORIENTATION, 0.1));
}

TEST(HasConverged, ForwardHeadingUsesPositionAndYawOnly) {
    // Same position and yaw, but different roll (single axis keeps error
    // purely in x-component, so z-component of quaternion_error is zero)
    Eigen::Quaterniond q_measured =
        Eigen::AngleAxisd(0.1, Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(0.5, Eigen::Vector3d::UnitX());
    Eigen::Quaterniond q_reference(
        Eigen::AngleAxisd(0.1, Eigen::Vector3d::UnitZ()));

    Pose measured =
        Pose::from_eigen(Eigen::Vector3d(1.0, 2.0, 3.0), q_measured);
    Pose reference =
        Pose::from_eigen(Eigen::Vector3d(1.0, 2.0, 3.0), q_reference);

    // Roll differs but should be ignored in FORWARD_HEADING mode
    EXPECT_TRUE(
        has_converged(measured, reference, WaypointMode::FORWARD_HEADING, 0.1));
}

// --- has_converged refactor equivalence ---

namespace {

// Verbatim copy of has_converged as it was before the controlled_error
// refactor. The refactor must not change any result.
bool legacy_has_converged(const Pose& state,
                          const Pose& waypoint_goal,
                          WaypointMode mode,
                          double convergence_threshold) {
    const Eigen::Vector3d ep = state.pos_vector() - waypoint_goal.pos_vector();
    const Eigen::Vector3d ea = vortex::utils::math::quaternion_error(
        state.ori_quaternion(), waypoint_goal.ori_quaternion());

    const double err = [&] {
        switch (mode) {
            case WaypointMode::ONLY_POSITION:
                return ep.norm();
            case WaypointMode::ONLY_ORIENTATION:
                return ea.norm();
            case WaypointMode::FORWARD_HEADING:
                return std::sqrt(ep.squaredNorm() + ea(2) * ea(2));
            case WaypointMode::POSITION_AND_YAW:
                return std::sqrt(ep.squaredNorm() + ea(2) * ea(2));
            case WaypointMode::XY_AND_YAW:
                return std::sqrt(ep.head<2>().squaredNorm() + ea(2) * ea(2));
            case WaypointMode::XY_FORWARD_DIR:
                return ep.head<2>().norm();
            case WaypointMode::LEVEL_ORIENTATION:
                return ea.head<2>().norm();
            case WaypointMode::ONLY_Z:
                return std::abs(ep(2));
            case WaypointMode::POS_Z_LEVEL_ORIENTATION:
                return std::sqrt(ep(2) * ep(2) + ea.head<2>().squaredNorm());
            case WaypointMode::FULL_POSE:
            default:
                return std::sqrt(ep.squaredNorm() + ea.squaredNorm());
        }
    }();
    return err < convergence_threshold;
}

Pose random_pose(std::mt19937& rng, double pos_range, double ang_range) {
    std::uniform_real_distribution<double> pos(-pos_range, pos_range);
    std::uniform_real_distribution<double> ang(-ang_range, ang_range);
    const Eigen::Quaterniond q =
        Eigen::AngleAxisd(ang(rng), Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(ang(rng), Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(ang(rng), Eigen::Vector3d::UnitX());
    return Pose::from_eigen(Eigen::Vector3d(pos(rng), pos(rng), pos(rng)),
                            q.normalized());
}

constexpr WaypointMode kAllModes[] = {
    WaypointMode::FULL_POSE,        WaypointMode::ONLY_POSITION,
    WaypointMode::FORWARD_HEADING,  WaypointMode::ONLY_ORIENTATION,
    WaypointMode::POSITION_AND_YAW, WaypointMode::XY_AND_YAW,
    WaypointMode::XY_FORWARD_DIR,   WaypointMode::LEVEL_ORIENTATION,
    WaypointMode::ONLY_Z,           WaypointMode::POS_Z_LEVEL_ORIENTATION};

}  // namespace

TEST(HasConvergedRefactor, MatchesLegacyForRandomPoses) {
    std::mt19937 rng(12345);
    std::uniform_real_distribution<double> thr(0.05, 1.5);
    int checked = 0;
    for (int i = 0; i < 10000; ++i) {
        // Small ranges so a good share of the cases lies near the threshold.
        const Pose goal = random_pose(rng, 0.5, 0.4);
        const Pose state = random_pose(rng, 0.5, 0.4);
        const double threshold = thr(rng);
        for (const auto mode : kAllModes) {
            EXPECT_EQ(has_converged(state, goal, mode, threshold),
                      legacy_has_converged(state, goal, mode, threshold))
                << "sample " << i << " mode " << static_cast<int>(mode);
            ++checked;
        }
    }
    EXPECT_EQ(checked, 100000);
}

// --- controlled_error and separate tolerances ---

TEST(ControlledError, SplitsPositionAndOrientation) {
    const Eigen::Quaterniond q(
        Eigen::AngleAxisd(0.2, Eigen::Vector3d::UnitZ()));
    const Pose state = Pose::from_eigen(Eigen::Vector3d(0.3, 0.4, 5.0), q);
    const Pose goal{};

    const auto full = controlled_error(state, goal, WaypointMode::FULL_POSE);
    EXPECT_NEAR(full.position, std::sqrt(0.09 + 0.16 + 25.0), 1e-12);
    EXPECT_NEAR(full.orientation, 0.2, 1e-3);

    const auto xy = controlled_error(state, goal, WaypointMode::XY_AND_YAW);
    EXPECT_NEAR(xy.position, 0.5, 1e-12);
    EXPECT_NEAR(xy.orientation, 0.2, 1e-3);

    const auto only_z = controlled_error(state, goal, WaypointMode::ONLY_Z);
    EXPECT_NEAR(only_z.position, 5.0, 1e-12);
    EXPECT_DOUBLE_EQ(only_z.orientation, 0.0);
}

TEST(HasConvergedTolerance, PositionAndOrientationCheckedSeparately) {
    const Eigen::Quaterniond q(
        Eigen::AngleAxisd(0.2, Eigen::Vector3d::UnitZ()));
    const Pose state = Pose::from_eigen(Eigen::Vector3d(0.05, 0.0, 0.0), q);
    const Pose goal{};

    // Position 5 cm, orientation ~0.2 rad.
    EXPECT_TRUE(has_converged(state, goal, WaypointMode::FULL_POSE,
                              ConvergenceTolerance{0.1, 0.3}));
    EXPECT_FALSE(has_converged(state, goal, WaypointMode::FULL_POSE,
                               ConvergenceTolerance{0.1, 0.1}));
    EXPECT_FALSE(has_converged(state, goal, WaypointMode::FULL_POSE,
                               ConvergenceTolerance{0.01, 0.3}));
    // The combined threshold would accept this, the tight orientation does not.
    EXPECT_TRUE(has_converged(state, goal, WaypointMode::FULL_POSE, 0.5));
}

TEST(HasConvergedTolerance, UnsetToleranceIsNotChecked) {
    const Pose state{5.0, 5.0, 5.0, 1.0, 0.0, 0.0, 0.0};
    const Pose goal{};
    EXPECT_TRUE(has_converged(state, goal, WaypointMode::FULL_POSE,
                              ConvergenceTolerance{0.0, 0.0}));
    EXPECT_TRUE(has_converged(state, goal, WaypointMode::ONLY_ORIENTATION,
                              ConvergenceTolerance{0.01, 0.01}));
}

// --- YAML loader: hold and separate tolerances ---

TEST(LoadWaypointGoal, ReadsHoldAndTolerances) {
    const auto path = std::filesystem::temp_directory_path() /
                      "vortex_utils_test_waypoint_goal.yaml";
    {
        std::ofstream out(path);
        out << "with_tolerances:\n"
               "  mode: full_pose\n"
               "  position: {x: 1.0, y: 2.0, z: 3.0}\n"
               "  orientation: {roll: 0.0, pitch: 0.0, yaw: 90.0}\n"
               "  hold_time: 1.5\n"
               "  position_tolerance: 0.07\n"
               "  orientation_tolerance_deg: 10.0\n"
               "plain:\n"
               "  mode: only_position\n"
               "  position: {x: 1.0, y: 2.0, z: 3.0}\n";
    }

    const auto goal =
        load_waypoint_goal_from_yaml(path.string(), "with_tolerances");
    EXPECT_DOUBLE_EQ(goal.hold_time_sec, 1.5);
    EXPECT_DOUBLE_EQ(goal.position_tolerance, 0.07);
    EXPECT_NEAR(goal.orientation_tolerance, 10.0 * M_PI / 180.0, 1e-12);

    const auto plain = load_waypoint_goal_from_yaml(path.string(), "plain");
    EXPECT_DOUBLE_EQ(plain.hold_time_sec, 0.0);
    EXPECT_DOUBLE_EQ(plain.position_tolerance, 0.0);
    EXPECT_DOUBLE_EQ(plain.orientation_tolerance, 0.0);

    std::filesystem::remove(path);
}

}  // namespace vortex::utils::waypoints
