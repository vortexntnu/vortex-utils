#include <gtest/gtest.h>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "vortex/utils/ros_transforms.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    ::testing::InitGoogleTest(&argc, argv);
    int ret = RUN_ALL_TESTS();
    rclcpp::shutdown();
    return ret;
}

class RosTransformsTest : public ::testing::Test {
   protected:
    void SetUp() override {
        node_ = std::make_shared<rclcpp::Node>("ros_transforms_test_node");

        buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
        buffer_->setUsingDedicatedThread(true);
        listener_ =
            std::make_unique<tf2_ros::TransformListener>(*buffer_, node_);
        broadcaster_ =
            std::make_unique<tf2_ros::StaticTransformBroadcaster>(node_);

        publish_static_transform();

        // Give TF some time to populate
        rclcpp::spin_some(node_);
        rclcpp::spin_some(node_);
    }

    void publish_static_transform() {
        geometry_msgs::msg::TransformStamped tf;
        tf.header.stamp = node_->get_clock()->now();
        tf.header.frame_id = "source";
        tf.child_frame_id = "target";

        tf.transform.translation.x = 1.0;
        tf.transform.translation.y = 2.0;
        tf.transform.translation.z = 3.0;

        tf.transform.rotation.w = 1.0;
        tf.transform.rotation.x = 0.0;
        tf.transform.rotation.y = 0.0;
        tf.transform.rotation.z = 0.0;

        broadcaster_->sendTransform(tf);
    }

    rclcpp::Logger logger_ = rclcpp::get_logger("ros_transforms_test");

    std::shared_ptr<rclcpp::Node> node_;
    std::unique_ptr<tf2_ros::Buffer> buffer_;
    std::unique_ptr<tf2_ros::TransformListener> listener_;
    std::unique_ptr<tf2_ros::StaticTransformBroadcaster> broadcaster_;
};

TEST_F(RosTransformsTest, pose_stamped_success) {
    geometry_msgs::msg::PoseStamped in, out;
    in.header.frame_id = "source";
    in.header.stamp = node_->get_clock()->now();

    in.pose.position.x = 1.0;
    in.pose.position.y = 1.0;
    in.pose.position.z = 1.0;
    in.pose.orientation.w = 1.0;

    bool ok = vortex::utils::ros_transforms::transform_pose(
        *buffer_, in, "target", out, logger_);

    ASSERT_TRUE(ok);
    EXPECT_EQ(out.header.frame_id, "target");

    EXPECT_NEAR(out.pose.position.x, 0.0, 1e-6);
    EXPECT_NEAR(out.pose.position.y, -1.0, 1e-6);
    EXPECT_NEAR(out.pose.position.z, -2.0, 1e-6);
}

TEST_F(RosTransformsTest, pose_stamped_failure) {
    geometry_msgs::msg::PoseStamped in, out;
    in.header.frame_id = "unknown_frame";
    in.header.stamp = node_->get_clock()->now();

    bool ok = vortex::utils::ros_transforms::transform_pose(
        *buffer_, in, "target", out, logger_);

    EXPECT_FALSE(ok);
}

TEST_F(RosTransformsTest, pose_with_covariance_success) {
    geometry_msgs::msg::PoseWithCovarianceStamped in, out;
    in.header.frame_id = "source";
    in.header.stamp = node_->get_clock()->now();

    in.pose.pose.position.x = 0.0;
    in.pose.pose.position.y = 0.0;
    in.pose.pose.position.z = 0.0;
    in.pose.pose.orientation.w = 1.0;

    // Simple diagonal covariance
    in.pose.covariance[0] = 0.1;
    in.pose.covariance[7] = 0.1;
    in.pose.covariance[14] = 0.1;

    bool ok = vortex::utils::ros_transforms::transform_pose(
        *buffer_, in, "target", out, logger_);

    ASSERT_TRUE(ok);
    EXPECT_EQ(out.header.frame_id, "target");

    EXPECT_NEAR(out.pose.pose.position.x, -1.0, 1e-6);
    EXPECT_NEAR(out.pose.pose.position.y, -2.0, 1e-6);
    EXPECT_NEAR(out.pose.pose.position.z, -3.0, 1e-6);
}

TEST_F(RosTransformsTest, pose_array_success) {
    geometry_msgs::msg::PoseArray in, out;
    in.header.frame_id = "source";
    in.header.stamp = node_->get_clock()->now();

    in.poses.resize(2);
    in.poses[0].position.x = 0.0;
    in.poses[1].position.x = 1.0;

    bool ok = vortex::utils::ros_transforms::transform_pose(
        *buffer_, in, "target", out, logger_);

    ASSERT_TRUE(ok);
    ASSERT_EQ(out.poses.size(), 2);

    EXPECT_NEAR(out.poses[0].position.x, -1.0, 1e-6);
    EXPECT_NEAR(out.poses[1].position.x, 0.0, 1e-6);
}

TEST_F(RosTransformsTest, pose_array_failure) {
    geometry_msgs::msg::PoseArray in, out;
    in.header.frame_id = "unknown_frame";
    in.header.stamp = node_->get_clock()->now();

    in.poses.resize(1);

    bool ok = vortex::utils::ros_transforms::transform_pose(
        *buffer_, in, "target", out, logger_);

    EXPECT_FALSE(ok);
}
