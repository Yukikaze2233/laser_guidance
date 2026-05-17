#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "ws30_lidar/client.hpp"

namespace {

using ws30_lidar::Client;
using ws30_lidar::ClientConfig;
using ws30_lidar::DeviceInfo;
using ws30_lidar::ImuSample;
using ws30_lidar::PointFrame;

auto make_point_cloud2(const PointFrame& frame,
                       const std::string& frame_id,
                       const rclcpp::Time& stamp) -> sensor_msgs::msg::PointCloud2 {
    sensor_msgs::msg::PointCloud2 msg;
    msg.header.frame_id = frame_id;
    msg.header.stamp = stamp;
    msg.height = 1;
    msg.width = static_cast<std::uint32_t>(frame.points.size());
    msg.is_dense = false;
    msg.is_bigendian = false;

    constexpr std::uint32_t kPointStep = 16;
    msg.point_step = kPointStep;
    msg.row_step = msg.point_step * msg.width;

    auto add_field = [&](const char* name, std::uint32_t offset, std::uint8_t datatype) {
        sensor_msgs::msg::PointField f;
        f.name = name;
        f.offset = offset;
        f.datatype = datatype;
        f.count = 1;
        msg.fields.push_back(f);
    };
    add_field("x", 0, sensor_msgs::msg::PointField::FLOAT32);
    add_field("y", 4, sensor_msgs::msg::PointField::FLOAT32);
    add_field("z", 8, sensor_msgs::msg::PointField::FLOAT32);
    add_field("intensity", 12, sensor_msgs::msg::PointField::FLOAT32);

    msg.data.resize(msg.row_step);
    auto* out = reinterpret_cast<float*>(msg.data.data());
    for (std::size_t i = 0; i < frame.points.size(); ++i) {
        const auto& p = frame.points[i];
        *out++ = p.x_m;
        *out++ = p.y_m;
        *out++ = p.z_m;
        *out++ = static_cast<float>(p.intensity);
    }

    return msg;
}

auto make_diagnostics(const DeviceInfo& info,
                      bool streaming,
                      std::uint64_t last_points_ts_ms,
                      std::uint64_t last_imu_ts_ms) -> diagnostic_msgs::msg::DiagnosticArray {
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = rclcpp::Clock().now();

    auto add_status = [&](const std::string& name) -> diagnostic_msgs::msg::DiagnosticStatus& {
        array.status.emplace_back();
        auto& s = array.status.back();
        s.name = name;
        s.hardware_id = "ws30_lidar";
        return s;
    };

    {
        auto& s = add_status("ws30_lidar: connection");
        if (info.connected.has_value() && *info.connected) {
            s.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
            s.message = "connected";
        } else if (info.serial_number.has_value()) {
            s.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
            s.message = "SN: " + *info.serial_number;
        } else {
            s.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
            s.message = "waiting for device info";
        }
    }

    {
        auto& s = add_status("ws30_lidar: streaming");
        if (streaming) {
            s.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
            s.message = "active";
        } else {
            s.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
            s.message = "inactive";
        }
    }

    {
        auto& s = add_status("ws30_lidar: points_age");
        if (last_points_ts_ms > 0) {
            const auto age_ms = static_cast<std::uint64_t>(
                rclcpp::Clock().now().nanoseconds() / 1'000'000) - last_points_ts_ms;
            s.level = age_ms < 500
                ? diagnostic_msgs::msg::DiagnosticStatus::OK
                : diagnostic_msgs::msg::DiagnosticStatus::WARN;
            s.message = std::to_string(age_ms) + " ms since last frame";
        } else {
            s.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
            s.message = "no points received yet";
        }
    }

    return array;
}

auto make_imu_msg(const ImuSample& imu,
                  const std::string& frame_id,
                  const rclcpp::Time& stamp) -> sensor_msgs::msg::Imu {
    sensor_msgs::msg::Imu msg;
    msg.header.frame_id = frame_id;
    msg.header.stamp = stamp;
    msg.angular_velocity.x = static_cast<double>(imu.gyro_x);
    msg.angular_velocity.y = static_cast<double>(imu.gyro_y);
    msg.angular_velocity.z = static_cast<double>(imu.gyro_z);
    msg.linear_acceleration.x = static_cast<double>(imu.acc_x);
    msg.linear_acceleration.y = static_cast<double>(imu.acc_y);
    msg.linear_acceleration.z = static_cast<double>(imu.acc_z);
    return msg;
}

}

class Ws30LidarNode : public rclcpp::Node {
public:
    Ws30LidarNode()
        : Node("ws30_lidar_node") {
        this->declare_parameter("device_ip", "192.168.137.200");
        this->declare_parameter("points_port", 1001);
        this->declare_parameter("imu_port", 1002);
        this->declare_parameter("status_port", 1003);
        this->declare_parameter("frame_id", "ws30_lidar");
        this->declare_parameter("imu_frame_id", "ws30_imu");
        this->declare_parameter("packet_timeout_ms", 50);
        this->declare_parameter("publish_imu", true);
        this->declare_parameter("use_sensor_timestamp", true);

        points_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "~/points", rclcpp::SensorDataQoS());
        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>(
            "~/imu", rclcpp::SensorDataQoS());
        diag_pub_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
            "~/status", rclcpp::QoS(1));

        reopen_srv_ = this->create_service<std_srvs::srv::Trigger>(
            "~/reopen",
            [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                   std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
                reopen();
                response->success = true;
                response->message = "reopen complete";
            });

        open_client();

        const auto points_period = std::chrono::milliseconds(
            std::max<std::int64_t>(1, this->get_parameter("packet_timeout_ms").as_int()));
        points_timer_ = this->create_wall_timer(
            points_period, std::bind(&Ws30LidarNode::poll_points, this));
        imu_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(2), std::bind(&Ws30LidarNode::poll_imu, this));
        diag_timer_ = this->create_wall_timer(
            std::chrono::seconds(1), std::bind(&Ws30LidarNode::publish_diagnostics, this));

        RCLCPP_INFO(this->get_logger(), "WS30 lidar bridge started");
        RCLCPP_INFO(this->get_logger(), "  device_ip: %s",
                    this->get_parameter("device_ip").as_string().c_str());
    }

    ~Ws30LidarNode() override {
        shutdown_client();
    }

private:
    void open_client() {
        ClientConfig cfg;
        cfg.device_ip = this->get_parameter("device_ip").as_string();
        cfg.points_port = static_cast<std::uint16_t>(this->get_parameter("points_port").as_int());
        cfg.imu_port = static_cast<std::uint16_t>(this->get_parameter("imu_port").as_int());
        cfg.status_port = static_cast<std::uint16_t>(this->get_parameter("status_port").as_int());
        cfg.receive_timeout_ms = this->get_parameter("packet_timeout_ms").as_int();

        client_ = std::make_unique<Client>(cfg);
        if (auto result = client_->open(); !result) {
            RCLCPP_ERROR(this->get_logger(), "failed to open WS30 client: %s",
                         result.error().c_str());
            client_.reset();
            return;
        }
        streaming_ = true;

        if (auto r = client_->request_points_stream(true); !r) {
            RCLCPP_WARN(this->get_logger(), "failed to request points: %s", r.error().c_str());
        }
        if (this->get_parameter("publish_imu").as_bool()) {
            if (auto r = client_->request_imu_stream(true); !r) {
                RCLCPP_WARN(this->get_logger(), "failed to request imu: %s", r.error().c_str());
            }
        }
        if (auto r = client_->request_serial_number(); !r) {
            RCLCPP_WARN(this->get_logger(), "failed to request SN: %s", r.error().c_str());
        }
    }

    void shutdown_client() {
        if (!client_) return;
        if (streaming_) {
            (void)client_->request_points_stream(false);
            if (this->get_parameter("publish_imu").as_bool()) {
                (void)client_->request_imu_stream(false);
            }
            streaming_ = false;
        }
        client_->close();
        client_.reset();
    }

    void reopen() {
        RCLCPP_INFO(this->get_logger(), "reopening WS30 client");
        shutdown_client();
        open_client();
    }

    void poll_points() {
        if (!client_ || !streaming_) return;

        const auto result = client_->poll_points_frame();
        if (!result) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                 "points poll error: %s", result.error().c_str());
            return;
        }
        if (!result->has_value()) return;

        const auto& frame = **result;
        last_points_ts_ms_ = frame.timestamp_ms;

        auto msg = make_point_cloud2(frame,
                                     this->get_parameter("frame_id").as_string(),
                                     this->now());
        if (this->get_parameter("use_sensor_timestamp").as_bool() && frame.timestamp_ms > 0) {
            msg.header.stamp = rclcpp::Time(
                static_cast<std::int64_t>(frame.timestamp_ms) * 1'000'000);
        }
        points_pub_->publish(msg);
    }

    void poll_imu() {
        if (!client_ || !streaming_) return;
        if (!this->get_parameter("publish_imu").as_bool()) return;

        const auto result = client_->poll_imu_sample();
        if (!result) return;
        if (!result->has_value()) return;

        const auto& imu = **result;
        last_imu_ts_ms_ = imu.timestamp_ms;

        auto msg = make_imu_msg(imu,
                                this->get_parameter("imu_frame_id").as_string(),
                                this->now());
        if (this->get_parameter("use_sensor_timestamp").as_bool() && imu.timestamp_ms > 0) {
            msg.header.stamp = rclcpp::Time(
                static_cast<std::int64_t>(imu.timestamp_ms) * 1'000'000);
        }
        imu_pub_->publish(msg);
    }

    void publish_diagnostics() {
        DeviceInfo info;
        if (client_) {
            const auto result = client_->poll_device_info();
            if (result && result->has_value()) {
                info = **result;
            }
        }
        auto diag = make_diagnostics(info, streaming_,
                                     last_points_ts_ms_, last_imu_ts_ms_);
        diag_pub_->publish(diag);
    }

    std::unique_ptr<Client> client_;
    bool streaming_ = false;
    std::uint64_t last_points_ts_ms_ = 0;
    std::uint64_t last_imu_ts_ms_ = 0;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr points_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_pub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reopen_srv_;

    rclcpp::TimerBase::SharedPtr points_timer_;
    rclcpp::TimerBase::SharedPtr imu_timer_;
    rclcpp::TimerBase::SharedPtr diag_timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Ws30LidarNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
