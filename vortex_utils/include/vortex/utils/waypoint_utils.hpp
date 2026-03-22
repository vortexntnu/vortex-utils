#ifndef VORTEX_UTILS_WAYPOINT_UTILS_HPP
#define VORTEX_UTILS_WAYPOINT_UTILS_HPP

#include <string>
#include <vortex/utils/types.hpp>

namespace vortex::utils::waypoints {

using vortex::utils::types::Pose;
using vortex::utils::types::WaypointMode;

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
 * @brief Load a waypoint from a YAML file by identifier.
 *
 * Expected YAML format:
 * @code
 * waypoint_name:
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
 * @param identifier The waypoint key to look up.
 * @return Pose with position and orientation (RPY converted to quaternion).
 * @throws std::runtime_error if the file cannot be opened or the identifier is
 * not found.
 */
Pose load_waypoint_from_yaml(const std::string& file_path,
                             const std::string& identifier);

}  // namespace vortex::utils::waypoints

#endif  // VORTEX_UTILS_WAYPOINT_UTILS_HPP
