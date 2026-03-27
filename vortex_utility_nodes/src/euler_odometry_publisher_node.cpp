#include <vortex/utils/math.hpp>
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "vortex_msgs/msg/pose_euler_stamped.hpp"

class EulerOdometryPublisherNode : public rclcpp::Node {
   public:
    EulerOdometryPublisherNode() : Node("euler_odometry_publisher_node") {
        this->declare_parameter<std::string>("topics.input_odom",
                                             "nautilus/odom");
        this->declare_parameter<std::string>("topics.output_euler",
                                             "utils/odometry/euler");

        auto input_topic = this->get_parameter("topics.input_odom").as_string();
        auto output_topic =
            this->get_parameter("topics.output_euler").as_string();

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            input_topic, 2,
            std::bind(&EulerOdometryPublisherNode::odom_callback, this,
                      std::placeholders::_1));

        euler_odom_pub_ =
            this->create_publisher<vortex_msgs::msg::PoseEulerStamped>(
                output_topic, 2);

        RCLCPP_INFO(this->get_logger(),
                    "Euler odometry publisher started: %s -> %s",
                    input_topic.c_str(), output_topic.c_str());
    }

   private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        vortex_msgs::msg::PoseEulerStamped euler_msg;

        const auto& q = msg->pose.pose.orientation;
        Eigen::Quaterniond eigen_q(q.w, q.x, q.y, q.z);
        auto euler = vortex::utils::math::quat_to_euler(eigen_q);

        euler_msg.x = msg->pose.pose.position.x;
        euler_msg.y = msg->pose.pose.position.y;
        euler_msg.z = msg->pose.pose.position.z;
        euler_msg.roll = euler.x();
        euler_msg.pitch = euler.y();
        euler_msg.yaw = euler.z();

        euler_odom_pub_->publish(euler_msg);
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<vortex_msgs::msg::PoseEulerStamped>::SharedPtr
        euler_odom_pub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<EulerOdometryPublisherNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
