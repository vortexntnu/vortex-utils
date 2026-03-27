#include <vortex/utils/math.hpp>
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"

class EulerOdometryPublisherNode : public rclcpp::Node {
   public:
    EulerOdometryPublisherNode() : Node("euler_odometry_publisher_node") {
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "nautilus/odometry", 2,
            std::bind(&EulerOdometryPublisherNode::odom_callback, this,
                      std::placeholders::_1));

        euler_odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
            "utils/odometry/euler", 2);

        RCLCPP_INFO(this->get_logger(),
                    "Euler odometry publisher node started.");
    }

   private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        auto euler_msg = *msg;

        const auto& q = msg->pose.pose.orientation;

        // Convert geometry_msgs quaternion to Eigen quaternion
        Eigen::Quaterniond eigen_q(q.w, q.x, q.y, q.z);
        auto euler = vortex::utils::math::quat_to_euler(eigen_q);

        // TODO: add custom message for naming
        euler_msg.pose.pose.orientation.x = euler.x();
        euler_msg.pose.pose.orientation.y = euler.y();
        euler_msg.pose.pose.orientation.z = euler.z();
        euler_msg.pose.pose.orientation.w = 0.0;

        euler_odom_pub_->publish(euler_msg);
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr euler_odom_pub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<EulerOdometryPublisherNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
