#ifndef VORTEX_UTILS_WAYPOINT_UTILS_HPP
#define VORTEX_UTILS_WAYPOINT_UTILS_HPP

#include <string>
#include <vortex/utils/types.hpp>

namespace vortex::utils::waypoints {

using vortex::utils::types::Pose;
using vortex::utils::types::WaypointMode;

/**
 * @brief Struct to represent a waypoint goal, containing the target pose, the
 * waypoint mode, and the convergence threshold.
 */
struct WaypointGoal {
    Pose pose;
    WaypointMode mode;
    double convergence_threshold{0.1};
};

/**
 * @brief Struct to represent a landmark convergence goal, containing the
 * convergence offset from the landmark pose, the waypoint mode, and the
 * convergence threshold.
 */
struct LandmarkConvergenceGoal {
    Pose convergence_offset;
    WaypointMode mode;
    double convergence_threshold{0.1};
    double dead_reckoning_threshold{0.5};
    double track_loss_timeout_sec{10.0};
};

/**
 * @brief Convert a string to a WaypointMode enum value.
 *
 * Accepts both upper-case ("FULL_POSE") and lower-case ("full_pose") variants.
 *
 * @param str The string representation of the mode.
 * @return The corresponding WaypointMode.
 * @throws std::runtime_error if the string does not match any known mode.
 */
WaypointMode string_to_waypoint_mode(const std::string& str);

/**
 * @brief Compute the waypoint goal by applying the mode logic to the incoming
 * waypoint.
 *
 * For modes that don't control all DOFs, the uncontrolled components are
 * replaced with values from @p current_state.
 *
 * @param incoming_waypoint The incoming waypoint to compute goal from.
 * @param mode The waypoint mode.
 * @param current_state The current state pose.
 * @return The adjusted waypoint goal.
 */
Pose compute_waypoint_goal(const Pose& incoming_waypoint,
                           WaypointMode mode,
                           const Pose& current_state);

/**
 * @brief Check whether the state has converged to the waypoint goal.
 *
 * Only the DOFs relevant to the waypoint mode are included in the error norm.
 *
 * @param state The current state pose.
 * @param waypoint_goal The waypoint goal pose.
 * @param mode The waypoint mode.
 * @param convergence_threshold The maximum allowed error norm.
 * @return True if the error is below the threshold.
 */
bool has_converged(const Pose& state,
                   const Pose& waypoint_goal,
                   WaypointMode mode,
                   double convergence_threshold);

/**
 * @brief Apply a pose offset to a base pose.
 *
 * Position: p_target = p_base + p_offset
 * Orientation: q_target = (q_base * q_offset).normalized()
 *
 * @param base The base pose (e.g. a landmark pose).
 * @param offset The offset to apply.
 * @return The resulting target pose.
 */
Pose apply_pose_offset(const Pose& base, const Pose& offset);

/**
 * @brief Load a pose from a YAML file by identifier.
 *
 * Expected YAML format:
 * @code
 * pose_name:
 *   position:
 *     x: 1.0
 *     y: 0.0
 *     z: -0.5
 *   orientation:
 *     roll: 0.0
 *     pitch: 0.0
 *     yaw: 3.14159
 * @endcode
 *
 * @param file_path Path to the YAML file.
 * @param identifier The pose key to look up.
 * @return Pose with position and orientation (RPY converted to quaternion).
 * @throws std::runtime_error if the file cannot be opened or the identifier is
 * not found.
 */
Pose load_pose_from_yaml(const std::string& file_path,
                         const std::string& identifier);

/**
 * @brief Load a waypoint goal from a YAML file by identifier.
 *
 * Expected YAML format:
 * @code
 * waypoint_name:
 *   mode: only_position  # WaypointMode as string or integer (e.g. 1 /
 * "only_position") position:            # Required for FULL_POSE,
 * ONLY_POSITION, FORWARD_HEADING x: 1.0 y: 0.0 z: -0.5 orientation:         #
 * Required for FULL_POSE, ONLY_ORIENTATION roll: 0.0 pitch: 0.0 yaw: 3.14159
 *   convergence_threshold: 0.1  # Optional, default is 0.1
 * @endcode
 *
 * @param file_path Path to the YAML file.
 * @param identifier The waypoint key to look up.
 * @return WaypointGoal with pose, mode, and convergence threshold.
 * @throws std::runtime_error if the file cannot be opened or the identifier is
 * not found.
 */
WaypointGoal load_waypoint_goal_from_yaml(const std::string& file_path,
                                          const std::string& identifier);

/**
 * @brief Load a landmark convergence goal from a YAML file by identifier.
 *
 * The position/orientation fields represent the convergence offset from
 * the landmark pose.
 *
 * Expected YAML format:
 * @code
 * landmark_goal_name:
 *   mode: full_pose  # WaypointMode as string or integer (e.g. 0 / "full_pose")
 *   position:        # Required for FULL_POSE, ONLY_POSITION, FORWARD_HEADING
 *     x: 0.0
 *     y: 0.0
 *     z: -0.5
 *   orientation:     # Required for FULL_POSE, ONLY_ORIENTATION
 *     roll: 0.0
 *     pitch: 0.0
 *     yaw: 0.0
 *   convergence_threshold: 0.1  # Optional, default is 0.1
 *   dead_reckoning_threshold: 0.5  # Optional, default is 0.5
 * @endcode
 *
 * @param file_path Path to the YAML file.
 * @param identifier The landmark convergence key to look up.
 * @return LandmarkConvergenceGoal with convergence offset, mode,
 * convergence threshold, and dead-reckoning threshold.
 * @throws std::runtime_error if the file cannot be opened or the identifier is
 * not found.
 */
LandmarkConvergenceGoal load_landmark_goal_from_yaml(
    const std::string& file_path,
    const std::string& identifier);

}  // namespace vortex::utils::waypoints

#endif  // VORTEX_UTILS_WAYPOINT_UTILS_HPP
