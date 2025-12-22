#ifndef VORTEX_UTILS__ROS_TRANSFORMS_HPP_
#define VORTEX_UTILS__ROS_TRANSFORMS_HPP_

#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

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
 * @param logger ROS logger used for warning output
 * @param timeout Maximum duration to wait for a valid transform
 * @return true if the transform succeeded, false otherwise
 */
inline bool transform_pose(tf2_ros::Buffer& tf_buffer,
                           const geometry_msgs::msg::PoseStamped& in,
                           const std::string& target_frame,
                           geometry_msgs::msg::PoseStamped& out,
                           rclcpp::Logger logger,
                           tf2::Duration timeout = tf2::durationFromSec(0.00)) {
    try {
        tf_buffer.transform(in, out, target_frame, timeout);
        return true;
    } catch (const tf2::TransformException& ex) {
        RCLCPP_WARN(logger, "TF transform failed from '%s' to '%s': %s",
                    in.header.frame_id.c_str(), target_frame.c_str(),
                    ex.what());
        return false;
    }
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
 * @param logger ROS logger used for warning output
 * @param timeout Maximum duration to wait for a valid transform
 * @return true if the transform succeeded, false otherwise
 */
inline bool transform_pose(
    tf2_ros::Buffer& tf_buffer,
    const geometry_msgs::msg::PoseWithCovarianceStamped& in,
    const std::string& target_frame,
    geometry_msgs::msg::PoseWithCovarianceStamped& out,
    rclcpp::Logger logger,
    tf2::Duration timeout = tf2::durationFromSec(0.00)) {
    try {
        tf_buffer.transform(in, out, target_frame, timeout);
        return true;
    } catch (const tf2::TransformException& ex) {
        RCLCPP_WARN(logger, "TF transform failed from '%s' to '%s': %s",
                    in.header.frame_id.c_str(), target_frame.c_str(),
                    ex.what());
        return false;
    }
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
 * @param logger ROS logger used for warning output
 * @param timeout Maximum duration to wait for each pose transform
 * @return true if all poses were successfully transformed, false otherwise
 */
inline bool transform_pose(tf2_ros::Buffer& tf_buffer,
                           const geometry_msgs::msg::PoseArray& in,
                           const std::string& target_frame,
                           geometry_msgs::msg::PoseArray& out,
                           rclcpp::Logger logger,
                           tf2::Duration timeout = tf2::durationFromSec(0.0)) {
    out.poses.clear();
    out.poses.reserve(in.poses.size());

    out.header.stamp = in.header.stamp;
    out.header.frame_id = target_frame;

    geometry_msgs::msg::PoseStamped in_ps, out_ps;
    in_ps.header = in.header;

    for (const auto& pose : in.poses) {
        in_ps.pose = pose;

        try {
            tf_buffer.transform(in_ps, out_ps, target_frame, timeout);
            out.poses.push_back(out_ps.pose);
        } catch (const tf2::TransformException& ex) {
            RCLCPP_WARN(logger,
                        "TF transform failed for PoseArray element "
                        "from '%s' to '%s': %s",
                        in.header.frame_id.c_str(), target_frame.c_str(),
                        ex.what());
            return false;
        }
    }
    return true;
}

}  // namespace vortex::utils::ros_transforms

#endif  // VORTEX_UTILS__ROS_TRANSFORMS_HPP_
