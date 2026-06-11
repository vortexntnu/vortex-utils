#include <Eigen/Geometry>
#include <vector>
#include <vortex/utils/math.hpp>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "vortex_msgs/msg/pose_euler_stamped.hpp"
#include "vortex_msgs/msg/reference_filter_quat.hpp"
#include "vortex_msgs/msg/waypoint.hpp"

class RpyPublisherNode : public rclcpp::Node {
   public:
    RpyPublisherNode() : Node("rpy_publisher_node") {
        this->declare_parameter<std::vector<std::string>>("input_topics", {});
        this->declare_parameter<std::vector<std::string>>("output_topics", {});
        this->declare_parameter<std::vector<std::string>>("input_types", {});

        auto input_topics =
            this->get_parameter("input_topics").as_string_array();
        auto output_topics =
            this->get_parameter("output_topics").as_string_array();
        auto input_types = this->get_parameter("input_types").as_string_array();

        if (input_topics.size() != output_topics.size() ||
            input_topics.size() != input_types.size()) {
            RCLCPP_FATAL(this->get_logger(),
                         "input_topics, output_topics and input_types must all "
                         "have the same length");
            rclcpp::shutdown();
            return;
        }

        rclcpp::QoS qos(2);
        qos.best_effort();

        for (size_t i = 0; i < input_topics.size(); ++i) {
            auto pub =
                this->create_publisher<vortex_msgs::msg::PoseEulerStamped>(
                    output_topics[i], qos);
            publishers_.push_back(pub);
            wire_subscription(input_topics[i], input_types[i], pub, qos);
            RCLCPP_INFO(this->get_logger(), "[%s] %s -> %s",
                        input_types[i].c_str(), input_topics[i].c_str(),
                        output_topics[i].c_str());
        }
    }

   private:
    using PubPtr =
        rclcpp::Publisher<vortex_msgs::msg::PoseEulerStamped>::SharedPtr;

    Eigen::Vector3d to_euler(double w, double x, double y, double z) {
        return vortex::utils::math::quat_to_euler(
            Eigen::Quaterniond(w, x, y, z));
    }

    void publish(const PubPtr& pub,
                 const std_msgs::msg::Header& header,
                 double x,
                 double y,
                 double z,
                 const Eigen::Vector3d& euler) {
        vortex_msgs::msg::PoseEulerStamped msg;
        msg.header = header;
        msg.x = x;
        msg.y = y;
        msg.z = z;
        msg.roll = euler.x();
        msg.pitch = euler.y();
        msg.yaw = euler.z();
        pub->publish(msg);
    }

    void wire_subscription(const std::string& topic,
                           const std::string& type,
                           const PubPtr& pub,
                           const rclcpp::QoS& qos) {
        if (type == "odometry") {
            subs_.push_back(this->create_subscription<nav_msgs::msg::Odometry>(
                topic, qos, [this, pub](nav_msgs::msg::Odometry::SharedPtr m) {
                    const auto& p = m->pose.pose.position;
                    const auto& q = m->pose.pose.orientation;
                    publish(pub, m->header, p.x, p.y, p.z,
                            to_euler(q.w, q.x, q.y, q.z));
                }));
        } else if (type == "pose_stamped") {
            subs_.push_back(
                this->create_subscription<geometry_msgs::msg::PoseStamped>(
                    topic, qos,
                    [this, pub](geometry_msgs::msg::PoseStamped::SharedPtr m) {
                        const auto& p = m->pose.position;
                        const auto& q = m->pose.orientation;
                        publish(pub, m->header, p.x, p.y, p.z,
                                to_euler(q.w, q.x, q.y, q.z));
                    }));
        } else if (type == "pose_with_covariance_stamped") {
            subs_.push_back(this->create_subscription<
                            geometry_msgs::msg::PoseWithCovarianceStamped>(
                topic, qos,
                [this,
                 pub](geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr
                          m) {
                    const auto& p = m->pose.pose.position;
                    const auto& q = m->pose.pose.orientation;
                    publish(pub, m->header, p.x, p.y, p.z,
                            to_euler(q.w, q.x, q.y, q.z));
                }));
        } else if (type == "waypoint") {
            subs_.push_back(
                this->create_subscription<vortex_msgs::msg::Waypoint>(
                    topic, qos,
                    [this, pub](vortex_msgs::msg::Waypoint::SharedPtr m) {
                        const auto& p = m->pose.position;
                        const auto& q = m->pose.orientation;
                        std_msgs::msg::Header h;
                        h.stamp = this->now();
                        publish(pub, h, p.x, p.y, p.z,
                                to_euler(q.w, q.x, q.y, q.z));
                    }));
        } else if (type == "reference_filter") {
            subs_.push_back(this->create_subscription<
                            vortex_msgs::msg::ReferenceFilterQuat>(
                topic, qos,
                [this,
                 pub](vortex_msgs::msg::ReferenceFilterQuat::SharedPtr m) {
                    publish(pub, m->header, m->x, m->y, m->z,
                            to_euler(m->qw, m->qx, m->qy, m->qz));
                }));
        } else if (type == "imu") {
            subs_.push_back(this->create_subscription<sensor_msgs::msg::Imu>(
                topic, qos, [this, pub](sensor_msgs::msg::Imu::SharedPtr m) {
                    const auto& q = m->orientation;
                    std_msgs::msg::Header h = m->header;
                    publish(pub, h, 0.0, 0.0, 0.0,
                            to_euler(q.w, q.x, q.y, q.z));
                }));
        } else {
            RCLCPP_ERROR(
                this->get_logger(),
                "Unknown input_type '%s' for topic '%s'. Valid types: "
                "odometry, pose_stamped, pose_with_covariance_stamped, "
                "waypoint, reference_filter, imu",
                type.c_str(), topic.c_str());
        }
    }

    // Held as rclcpp::SubscriptionBase to store heterogeneous sub types.
    std::vector<rclcpp::SubscriptionBase::SharedPtr> subs_;
    std::vector<PubPtr> publishers_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RpyPublisherNode>());
    rclcpp::shutdown();
    return 0;
}
