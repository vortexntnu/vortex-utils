#include <optional>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/fluid_pressure.hpp"
#include "std_msgs/msg/float64.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

class PressureToDepthNode : public rclcpp::Node {
   public:
    PressureToDepthNode() : Node("pressure_to_depth") {
        this->declare_parameter<double>("atmospheric_pressure", 101500.0);
        this->declare_parameter<double>("water_density", 1000.0);
        this->declare_parameter<double>("gravity", 9.82841);
        this->declare_parameter<bool>("transform_to_base_link", false);

        atm_pressure_ = this->get_parameter("atmospheric_pressure").as_double();
        water_density_ = this->get_parameter("water_density").as_double();
        gravity_ = this->get_parameter("gravity").as_double();
        transform_to_base_link_ = this->get_parameter("transform_to_base_link").as_bool();

        RCLCPP_INFO(this->get_logger(),
                    "Atmospheric pressure: %.1f Pa  rho: %.1f kg/m^3  g: %.5f m/s^2",
                    atm_pressure_, water_density_, gravity_);

        if (transform_to_base_link_) {
            tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
            tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
            RCLCPP_INFO(this->get_logger(),
                        "Will apply TF: pressure_sensor_link -> base_link");
        }

        sub_ = this->create_subscription<sensor_msgs::msg::FluidPressure>(
            "pressure", 10,
            [this](const sensor_msgs::msg::FluidPressure::SharedPtr msg) { callback(msg); });

        pub_ = this->create_publisher<std_msgs::msg::Float64>("depth", 10);
    }

   private:
    // Looks up and caches the z offset from pressure_sensor_link to base_link.
    // Returns false if the TF is not yet available.
    bool ensure_tf() {
        if (sensor_to_base_z_.has_value()) return true;
        try {
            auto tf = tf_buffer_->lookupTransform(
                "nautilus/base_link", "nautilus/pressure_sensor_link", tf2::TimePointZero);
            sensor_to_base_z_ = tf.transform.translation.z;
            RCLCPP_INFO(this->get_logger(),
                        "Pressure sensor z offset from base_link: %.4f m", *sensor_to_base_z_);
            return true;
        } catch (const tf2::TransformException& ex) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "Waiting for pressure sensor TF: %s", ex.what());
            return false;
        }
    }

    void callback(const sensor_msgs::msg::FluidPressure::SharedPtr msg) {
        if (transform_to_base_link_ && !ensure_tf()) return;

        double depth = (msg->fluid_pressure - atm_pressure_) / (water_density_ * gravity_);

        // Shift depth from sensor frame to base_link: sensor is at z offset from base_link,
        // so base_link depth = sensor depth - z_offset (negative z = sensor is above base_link).
        if (sensor_to_base_z_.has_value()) {
            depth -= *sensor_to_base_z_;
        }

        std_msgs::msg::Float64 out;
        out.data = depth;
        pub_->publish(out);
    }

    double atm_pressure_;
    double water_density_;
    double gravity_;
    bool transform_to_base_link_;

    std::optional<double> sensor_to_base_z_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    rclcpp::Subscription<sensor_msgs::msg::FluidPressure>::SharedPtr sub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PressureToDepthNode>());
    rclcpp::shutdown();
    return 0;
}
