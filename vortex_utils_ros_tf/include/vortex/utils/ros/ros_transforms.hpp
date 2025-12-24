#ifndef VORTEX_UTILS__ROS_TRANSFORMS_HPP_
#define VORTEX_UTILS__ROS_TRANSFORMS_HPP_

#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <vortex_msgs/msg/landmark_array.hpp>

#include <tf2/exceptions.h>
#include <tf2/time.h>
#include <tf2_ros/buffer.h>

namespace vortex::utils::ros_transforms {

/**
 * @brief Transform a stamped pose into a target frame using TF.
 *
 * Applies a TF transform to a geometry_msgs::msg::PoseStamped message,
 * producing a pose expressed in the specified target frame.
 *
 * The input pose timestamp is used for TF lookup. If no valid transform
 * is available within the specified timeout, the transformation fails.
 *
 * @param tf_buffer TF buffer used for transform lookup
 * @param in Input PoseStamped message
 * @param target_frame Target frame ID
 * @param out Output PoseStamped message in the target frame
 * @param timeout Maximum duration to wait for a valid transform, defaults to 0
 * @note This function throws tf2::TransformException on failure.
 *       Callers are expected to handle errors via try/catch.
 */
inline void transform_pose(tf2_ros::Buffer& tf_buffer,
                           const geometry_msgs::msg::PoseStamped& in,
                           const std::string& target_frame,
                           geometry_msgs::msg::PoseStamped& out,
                           tf2::Duration timeout = tf2::durationFromSec(0.00)) {
    tf_buffer.transform(in, out, target_frame, timeout);
}

/**
 * @brief Transform a stamped pose with covariance into a target frame using TF.
 *
 * Applies a TF transform to a geometry_msgs::msg::PoseWithCovarianceStamped
 * message. Both the pose and its 6×6 covariance matrix are transformed
 * according to the rigid-body frame transform.
 *
 * The covariance is rotated into the target frame but is not otherwise
 * modified (no uncertainty inflation or filtering is applied).
 *
 * @param tf_buffer TF buffer used for transform lookup
 * @param in Input PoseWithCovarianceStamped message
 * @param target_frame Target frame ID
 * @param out Output PoseWithCovarianceStamped message in the target frame
 * @param timeout Maximum duration to wait for a valid transform
 * @note This function throws tf2::TransformException on failure.
 *       Callers are expected to handle errors via try/catch.
 */
inline void transform_pose(
    tf2_ros::Buffer& tf_buffer,
    const geometry_msgs::msg::PoseWithCovarianceStamped& in,
    const std::string& target_frame,
    geometry_msgs::msg::PoseWithCovarianceStamped& out,
    tf2::Duration timeout = tf2::durationFromSec(0.00)) {
    tf_buffer.transform(in, out, target_frame, timeout);
}

/**
 * @brief Transform all poses in a PoseArray into a target frame using TF.
 *
 * Each pose in the input PoseArray is individually transformed using the
 * array header (frame ID and timestamp). The output PoseArray contains
 * only successfully transformed poses and is expressed in the target frame.
 *
 * TF does not natively support PoseArray, so each pose is internally
 * wrapped as a PoseStamped for transformation.
 *
 * @param tf_buffer TF buffer used for transform lookup
 * @param in Input PoseArray message
 * @param target_frame Target frame ID
 * @param out Output PoseArray message in the target frame
 * @param timeout Maximum duration to wait for each pose transform
 * @note This function throws tf2::TransformException on failure.
 *       Callers are expected to handle errors via try/catch.
 */
inline void transform_pose(tf2_ros::Buffer& tf_buffer,
                           const geometry_msgs::msg::PoseArray& in,
                           const std::string& target_frame,
                           geometry_msgs::msg::PoseArray& out,
                           tf2::Duration timeout = tf2::durationFromSec(0.0)) {
    out.poses.clear();
    out.poses.reserve(in.poses.size());

    out.header.stamp = in.header.stamp;
    out.header.frame_id = target_frame;

    geometry_msgs::msg::PoseStamped in_ps, out_ps;
    in_ps.header = in.header;

    for (const auto& pose : in.poses) {
        in_ps.pose = pose;

        tf_buffer.transform(in_ps, out_ps, target_frame, timeout);
        out.poses.push_back(out_ps.pose);
    }
}

/**
 * @brief Transform all poses in a LandmarkArray into a target frame using TF.
 *
 * Each pose in the input LandmarkArray is individually transformed using the
 * array header (frame ID and timestamp). The output LandmarkArray contains
 * only successfully transformed poses and is expressed in the target frame.
 *
 * TF does not natively support PoseArray, so each pose is internally
 * wrapped as a PoseStamped for transformation.
 *
 * @param tf_buffer TF buffer used for transform lookup
 * @param in Input PoseArray message
 * @param target_frame Target frame ID
 * @param out Output PoseArray message in the target frame
 * @param timeout Maximum duration to wait for each pose transform
 * @note This function throws tf2::TransformException on failure.
 *       Callers are expected to handle errors via try/catch.
 */
inline void transform_pose(tf2_ros::Buffer& tf_buffer,
                           const vortex_msgs::msg::LandmarkArray& in,
                           const std::string& target_frame,
                           vortex_msgs::msg::LandmarkArray& out,
                           tf2::Duration timeout = tf2::durationFromSec(0.0)) {
    out.header.stamp = in.header.stamp;
    out.header.frame_id = target_frame;
    out.landmarks.clear();
    out.landmarks.reserve(in.landmarks.size());

    for (const auto& lm : in.landmarks) {
        vortex_msgs::msg::Landmark lm_out = lm;

        geometry_msgs::msg::PoseWithCovarianceStamped in_ps, out_ps;
        in_ps.header = in.header;
        in_ps.pose = lm.pose;

        tf_buffer.transform(in_ps, out_ps, target_frame, timeout);

        lm_out.pose = out_ps.pose;
        out.landmarks.push_back(lm_out);
    }
}

}  // namespace vortex::utils::ros_transforms

#endif  // VORTEX_UTILS__ROS_TRANSFORMS_HPP_
