#include <vortex/utils/math.hpp>
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "vortex_msgs/msg/reference_filter_quat.hpp"
#include "vortex_msgs/msg/rpy.hpp"
#include "vortex_msgs/msg/waypoint.hpp"

#include <variant>

class MessagePublisherNode : public rclcpp::Node {
   public:
    MessagePublisherNode() : Node("message_publisher_node") {
        this->declare_parameter<std::string>("input_type", "odometry");
        this->declare_parameter<std::string>("topics.odometry", "nautilus/odom");
        this->declare_parameter<std::string>("topics.waypoint",
                                             "nautilus/waypoint");
        this->declare_parameter<std::string>("topics.reference_filter",
                                             "nautilus/reference_filter");
        this->declare_parameter<std::string>("topics.output",
                                             "utils/odometry/euler");

        auto input_type = this->get_parameter("input_type").as_string();
        auto output_topic = this->get_parameter("topics.output").as_string();

        rpy_pub_ =
            this->create_publisher<vortex_msgs::msg::RPY>(output_topic, 2);

        if (input_type == "odometry") {
            auto topic = this->get_parameter("topics.odometry").as_string();
            sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
                topic, 2,
                std::bind(&MessagePublisherNode::odom_callback, this,
                          std::placeholders::_1));
            RCLCPP_INFO(this->get_logger(),
                        "Euler publisher [odometry]: %s -> %s", topic.c_str(),
                        output_topic.c_str());

        } else if (input_type == "waypoint") {
            auto topic = this->get_parameter("topics.waypoint").as_string();
            sub_ = this->create_subscription<vortex_msgs::msg::Waypoint>(
                topic, 2,
                std::bind(&MessagePublisherNode::waypoint_callback, this,
                          std::placeholders::_1));
            RCLCPP_INFO(this->get_logger(),
                        "Euler publisher [waypoint]: %s -> %s", topic.c_str(),
                        output_topic.c_str());

        } else if (input_type == "reference_filter") {
            auto topic =
                this->get_parameter("topics.reference_filter").as_string();
            sub_ = this->create_subscription<
                vortex_msgs::msg::ReferenceFilterQuat>(
                topic, 2,
                std::bind(&MessagePublisherNode::ref_filter_callback, this,
                          std::placeholders::_1));
            RCLCPP_INFO(this->get_logger(),
                        "Euler publisher [reference_filter]: %s -> %s",
                        topic.c_str(), output_topic.c_str());

        } else {
            RCLCPP_FATAL(this->get_logger(),
                         "Unknown input_type: '%s'. Must be 'odometry', "
                         "'waypoint', or 'reference_filter'.",
                         input_type.c_str());
            rclcpp::shutdown();
            return;
        }
    }

   private:
    Eigen::Vector3d convert_quat(double w, double x, double y, double z) {
        Eigen::Quaterniond eigen_q(w, x, y, z);
        return vortex::utils::math::quat_to_euler(eigen_q);
    }

    void publish_rpy(const std_msgs::msg::Header& header,
                     const Eigen::Vector3d& euler) {
        vortex_msgs::msg::RPY msg;
        msg.header = header;
        msg.roll = euler.x();
        msg.pitch = euler.y();
        msg.yaw = euler.z();
        rpy_pub_->publish(msg);
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        const auto& q = msg->pose.pose.orientation;
        auto euler = convert_quat(q.w, q.x, q.y, q.z);
        publish_rpy(msg->header, euler);
    }

    void waypoint_callback(const vortex_msgs::msg::Waypoint::SharedPtr msg) {
        const auto& q = msg->pose.orientation;
        auto euler = convert_quat(q.w, q.x, q.y, q.z);
        std_msgs::msg::Header header;
        header.stamp = this->now();
        publish_rpy(header, euler);
    }

    void ref_filter_callback(
        const vortex_msgs::msg::ReferenceFilterQuat::SharedPtr msg) {
        auto euler = convert_quat(msg->qw, msg->qx, msg->qy, msg->qz);
        publish_rpy(msg->header, euler);
    }

    using SubVariant = std::variant<
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr,
        rclcpp::Subscription<vortex_msgs::msg::Waypoint>::SharedPtr,
        rclcpp::Subscription<vortex_msgs::msg::ReferenceFilterQuat>::SharedPtr>;

    SubVariant sub_;
    rclcpp::Publisher<vortex_msgs::msg::RPY>::SharedPtr rpy_pub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MessagePublisherNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
