#include "vortex/utils/waypoint_utils.hpp"

#include <yaml-cpp/yaml.h>
#include <cmath>
#include <vortex/utils/math.hpp>

namespace vortex::utils::waypoints {

Pose compute_waypoint_goal(const Pose& incoming_waypoint,
                           WaypointMode mode,
                           const Pose& current_state) {
    Pose waypoint_out = incoming_waypoint;

    switch (mode) {
        case WaypointMode::FULL_POSE:
            break;

        case WaypointMode::ONLY_POSITION:
            waypoint_out.set_ori(current_state.ori_quaternion());
            break;

        case WaypointMode::FORWARD_HEADING: {
            double dx = incoming_waypoint.x - current_state.x;
            double dy = incoming_waypoint.y - current_state.y;
            double forward_heading = std::atan2(dy, dx);

            waypoint_out.set_ori(Eigen::Quaterniond(
                Eigen::AngleAxisd(forward_heading, Eigen::Vector3d::UnitZ())));
            break;
        }

        case WaypointMode::ONLY_ORIENTATION:
            waypoint_out.set_pos(current_state.pos_vector());
            break;
    }

    return waypoint_out;
}

bool has_converged(const Pose& state,
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
            case WaypointMode::FULL_POSE:
            default:
                return std::sqrt(ep.squaredNorm() + ea.squaredNorm());
        }
    }();

    return err < convergence_threshold;
}

Pose apply_pose_offset(const Pose& base, const Pose& offset) {
    const Eigen::Vector3d p_base = base.pos_vector();
    const Eigen::Quaterniond q_base = base.ori_quaternion().normalized();

    const Eigen::Vector3d p_offset = offset.pos_vector();
    const Eigen::Quaterniond q_offset = offset.ori_quaternion().normalized();

    const Eigen::Vector3d p_target = p_base + p_offset;
    const Eigen::Quaterniond q_target = (q_base * q_offset).normalized();

    return Pose::from_eigen(p_target, q_target);
}

Pose load_pose_from_yaml(const std::string& file_path,
                         const std::string& identifier) {
    YAML::Node root = YAML::LoadFile(file_path);

    if (!root[identifier]) {
        throw std::runtime_error("Pose '" + identifier + "' not found in " +
                                 file_path);
    }

    const auto& pose = root[identifier];

    const double x = pose["position"]["x"].as<double>();
    const double y = pose["position"]["y"].as<double>();
    const double z = pose["position"]["z"].as<double>();

    const double roll = pose["orientation"]["roll"].as<double>();
    const double pitch = pose["orientation"]["pitch"].as<double>();
    const double yaw = pose["orientation"]["yaw"].as<double>();

    const Eigen::Quaterniond q =
        vortex::utils::math::euler_to_quat(roll, pitch, yaw);

    return Pose{.x = x,
                .y = y,
                .z = z,
                .qw = q.w(),
                .qx = q.x(),
                .qy = q.y(),
                .qz = q.z()};
}

WaypointGoal load_waypoint_goal_from_yaml(const std::string& file_path,
                                          const std::string& identifier) {
    YAML::Node root = YAML::LoadFile(file_path);

    if (!root[identifier]) {
        throw std::runtime_error("Waypoint '" + identifier + "' not found in " +
                                 file_path);
    }

    const auto& wp = root[identifier];

    Pose pose = load_pose_from_yaml(file_path, identifier);

    const auto mode = static_cast<WaypointMode>(wp["mode"].as<uint8_t>());

    double convergence_threshold = 0.1;
    if (wp["convergence_threshold"]) {
        convergence_threshold = wp["convergence_threshold"].as<double>();
    }

    return WaypointGoal{.pose = pose,
                        .mode = mode,
                        .convergence_threshold = convergence_threshold};
}

LandmarkConvergenceGoal load_landmark_goal_from_yaml(
    const std::string& file_path,
    const std::string& identifier) {
    YAML::Node root = YAML::LoadFile(file_path);

    if (!root[identifier]) {
        throw std::runtime_error("Landmark goal '" + identifier +
                                 "' not found in " + file_path);
    }

    const auto& entry = root[identifier];

    Pose convergence_offset = load_pose_from_yaml(file_path, identifier);

    const auto mode = static_cast<WaypointMode>(entry["mode"].as<uint8_t>());

    double convergence_threshold = 0.1;
    if (entry["convergence_threshold"]) {
        convergence_threshold = entry["convergence_threshold"].as<double>();
    }

    double dead_reckoning_threshold = 0.5;
    if (entry["dead_reckoning_threshold"]) {
        dead_reckoning_threshold =
            entry["dead_reckoning_threshold"].as<double>();
    }

    return LandmarkConvergenceGoal{
        .convergence_offset = convergence_offset,
        .mode = mode,
        .convergence_threshold = convergence_threshold,
        .dead_reckoning_threshold = dead_reckoning_threshold};
}

}  // namespace vortex::utils::waypoints
