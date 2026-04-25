#include "vortex/utils/waypoint_utils.hpp"

#include <yaml-cpp/yaml.h>
#include <cmath>
#include <vortex/utils/math.hpp>

namespace vortex::utils::waypoints {

WaypointMode string_to_waypoint_mode(const std::string& str) {
    if (str == "FULL_POSE" || str == "full_pose")
        return WaypointMode::FULL_POSE;
    if (str == "ONLY_POSITION" || str == "only_position")
        return WaypointMode::ONLY_POSITION;
    if (str == "FORWARD_HEADING" || str == "forward_heading")
        return WaypointMode::FORWARD_HEADING;
    if (str == "ONLY_ORIENTATION" || str == "only_orientation")
        return WaypointMode::ONLY_ORIENTATION;
    if (str == "POSITION_AND_YAW" || str == "position_and_yaw")
        return WaypointMode::POSITION_AND_YAW;
    if (str == "XY_AND_YAW" || str == "xy_and_yaw")
        return WaypointMode::XY_AND_YAW;
    throw std::runtime_error("Unknown WaypointMode string: '" + str + "'");
}

namespace {

WaypointMode load_mode(const YAML::Node& node) {
    const auto& m = node["mode"];
    if (!m) {
        throw std::runtime_error("Missing required field 'mode'");
    }
    try {
        return static_cast<WaypointMode>(m.as<uint8_t>());
    } catch (const YAML::BadConversion&) {
        return string_to_waypoint_mode(m.as<std::string>());
    }
}

Pose load_pose_for_mode(const YAML::Node& node, WaypointMode mode) {  //
    Pose pose;  // defaults: x=y=z=0, qw=1, qx=qy=qz=0

    const bool needs_position = (mode == WaypointMode::FULL_POSE ||
                                 mode == WaypointMode::ONLY_POSITION ||
                                 mode == WaypointMode::FORWARD_HEADING ||
                                 mode == WaypointMode::POSITION_AND_YAW ||
                                 mode == WaypointMode::XY_AND_YAW);
    const bool needs_orientation = (mode == WaypointMode::FULL_POSE ||
                                    mode == WaypointMode::ONLY_ORIENTATION ||
                                    mode == WaypointMode::POSITION_AND_YAW ||
                                    mode == WaypointMode::XY_AND_YAW);

    if (needs_position) {
        pose.x = node["position"]["x"].as<double>();
        pose.y = node["position"]["y"].as<double>();
        pose.z = node["position"]["z"].as<double>();
    }

    if (needs_orientation) {
        const bool force_level = (mode == WaypointMode::POSITION_AND_YAW ||
                                  mode == WaypointMode::XY_AND_YAW);
        const double roll =
            force_level
                ? 0.0
                : node["orientation"]["roll"].as<double>() * (M_PI / 180.0);
        const double pitch =
            force_level
                ? 0.0
                : node["orientation"]["pitch"].as<double>() * (M_PI / 180.0);
        const double yaw =
            node["orientation"]["yaw"].as<double>() * (M_PI / 180.0);
        const Eigen::Quaterniond q =
            vortex::utils::math::euler_to_quat(roll, pitch, yaw);
        pose.qw = q.w();
        pose.qx = q.x();
        pose.qy = q.y();
        pose.qz = q.z();
    }

    return pose;
}

}  // namespace

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

        case WaypointMode::POSITION_AND_YAW: {
            const double raw_yaw = vortex::utils::math::quat_to_euler(
                incoming_waypoint.ori_quaternion())(2);
            const double current_yaw = vortex::utils::math::quat_to_euler(
                current_state.ori_quaternion())(2);
            const double yaw =
                current_yaw + vortex::utils::math::ssa(raw_yaw - current_yaw);
            waypoint_out.set_ori(Eigen::Quaterniond(
                Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ())));
            break;
        }

        case WaypointMode::XY_AND_YAW: {
            waypoint_out.z = current_state.z;
            const double raw_yaw = vortex::utils::math::quat_to_euler(
                incoming_waypoint.ori_quaternion())(2);
            const double current_yaw = vortex::utils::math::quat_to_euler(
                current_state.ori_quaternion())(2);
            const double yaw =
                current_yaw + vortex::utils::math::ssa(raw_yaw - current_yaw);
            waypoint_out.set_ori(Eigen::Quaterniond(
                Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ())));
            break;
        }
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
            case WaypointMode::POSITION_AND_YAW:
                return std::sqrt(ep.squaredNorm() + ea(2) * ea(2));
            case WaypointMode::XY_AND_YAW:
                return std::sqrt(ep.head<2>().squaredNorm() + ea(2) * ea(2));
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
                         const std::string& identifier) {  //
    YAML::Node root = YAML::LoadFile(file_path);

    if (!root[identifier]) {
        throw std::runtime_error("Pose '" + identifier + "' not found in " +
                                 file_path);
    }

    const auto& pose = root[identifier];

    const double x = pose["position"]["x"].as<double>();
    const double y = pose["position"]["y"].as<double>();
    const double z = pose["position"]["z"].as<double>();

    const double roll =
        pose["orientation"]["roll"].as<double>() * (M_PI / 180.0);
    const double pitch =
        pose["orientation"]["pitch"].as<double>() * (M_PI / 180.0);
    const double yaw = pose["orientation"]["yaw"].as<double>() * (M_PI / 180.0);

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

    const auto mode = load_mode(wp);
    const Pose pose = load_pose_for_mode(wp, mode);

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

    const auto mode = load_mode(entry);
    const Pose convergence_offset = load_pose_for_mode(entry, mode);

    double convergence_threshold = 0.1;
    if (entry["convergence_threshold"]) {
        convergence_threshold = entry["convergence_threshold"].as<double>();
    }

    double dead_reckoning_threshold = 0.5;
    if (entry["dead_reckoning_threshold"]) {
        dead_reckoning_threshold =
            entry["dead_reckoning_threshold"].as<double>();
    }

    double track_loss_timeout_sec = 10.0;
    if (entry["track_loss_timeout_sec"]) {
        track_loss_timeout_sec = entry["track_loss_timeout_sec"].as<double>();
    }

    return LandmarkConvergenceGoal{
        .convergence_offset = convergence_offset,
        .mode = mode,
        .convergence_threshold = convergence_threshold,
        .dead_reckoning_threshold = dead_reckoning_threshold,
        .track_loss_timeout_sec = track_loss_timeout_sec};
}

}  // namespace vortex::utils::waypoints
