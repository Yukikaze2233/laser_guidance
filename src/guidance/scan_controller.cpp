#include "guidance/scan_controller.hpp"

#include <chrono>
#include <print>
#include <thread>

#include "guidance/galvo_driver.hpp"
#include "guidance/galvo_executor.hpp"
#include "laser_guidance/error.hpp"

namespace rmcs_laser_guidance {

ScanController::ScanController(const GuidanceConfig& config, GalvoExecutor& executor)
    : config_(config)
    , executor_(executor)
    , enabled_(
          (config.scan_mode == ScanMode::rectangle || config.scan_mode == ScanMode::sine)
          && executor.is_initialized())
    , voltage_mode_(config.command_model == GuidanceCommandModelKind::direct_voltage) {
    if (enabled_) {
        worker_ = std::jthread([this] { run(); });
    }
}

ScanController::~ScanController() { stop(); }

auto ScanController::update_angles_center(const cv::Point2f& center) -> void {
    if (!enabled_ || voltage_mode_) {
        return;
    }
    {
        std::scoped_lock lock(mutex_);
        angle_center_ = center;
        active_ = true;
    }
    cv_.notify_one();
}

auto ScanController::update_voltage_center(const cv::Point2f& center) -> void {
    if (!enabled_ || !voltage_mode_) {
        return;
    }
    {
        std::scoped_lock lock(mutex_);
        voltage_center_ = center;
        active_ = true;
    }
    cv_.notify_one();
}

auto ScanController::deactivate() -> void {
    std::scoped_lock lock(mutex_);
    active_ = false;
}

auto ScanController::stop() -> void {
    {
        std::scoped_lock lock(mutex_);
        stop_requested_ = true;
        active_ = false;
    }
    worker_.request_stop();
    cv_.notify_all();
}

auto ScanController::run_pass(const std::vector<std::pair<float, float>>& path) -> std::string {
    if (path.empty()) {
        return {};
    }
    auto* driver = executor_.driver();
    if (driver == nullptr) {
        return "scan driver unavailable";
    }
    if (!(config_.scan_max_velocity_deg_s > 0.0F) || !(config_.scan_accel_deg_s2 > 0.0F)) {
        std::println(
            stderr,
            "guidance: invalid scan motion params (v={} a={}), jumping to path start",
            config_.scan_max_velocity_deg_s, config_.scan_accel_deg_s2);
        auto r = voltage_mode_ ? driver->set_voltages(path.front().first, path.front().second)
                               : driver->set_angles(path.front().first, path.front().second);
        return r ? std::string{} : format_error(r.error());
    }

    MotionPlanner planner(
        config_.scan_max_velocity_deg_s, config_.scan_accel_deg_s2, 0.001F);
    planner.set_origin(path.front().first, path.front().second);
    for (std::size_t i = 1; i < path.size(); ++i) {
        planner.move_to(path[i].first, path[i].second);
    }

    while (!planner.done()) {
        {
            std::scoped_lock lock(mutex_);
            if (stop_requested_ || !active_) {
                return {};
            }
        }
        const auto pt = planner.tick(0.001F);
        if (!pt) {
            break;
        }
        auto result = voltage_mode_ ? driver->set_voltages(pt->first, pt->second)
                                    : driver->set_angles(pt->first, pt->second);
        if (!result) {
            return "scan write failed: " + format_error(result.error());
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    return {};
}

auto ScanController::run() -> void {
    auto stoken = worker_.get_stop_token();
    while (!stoken.stop_requested()) {
        std::optional<cv::Point2f> angle_center;
        std::optional<cv::Point2f> voltage_center;
        {
            std::unique_lock lock(mutex_);
            cv_.wait(lock, [this, &stoken] {
                return stoken.stop_requested() || stop_requested_ || active_;
            });
            if (stoken.stop_requested()) {
                return;
            }
            angle_center = angle_center_;
            voltage_center = voltage_center_;
        }

        const bool sine = config_.scan_mode == ScanMode::sine;
        std::vector<std::pair<float, float>> path;
        if (voltage_mode_ && voltage_center.has_value()) {
            auto* driver = executor_.driver();
            if (driver == nullptr) {
                continue;
            }
            const float wv = driver->optical_to_voltage(config_.scan_width_deg);
            const float hv = driver->optical_to_voltage(config_.scan_height_deg);
            path = sine
                ? make_sine_path(
                      voltage_center->x, voltage_center->y, wv, hv, config_.scan_sine_cycles, 128)
                : make_rectangle_snake_path(
                      voltage_center->x, voltage_center->y, wv, hv, config_.scan_grid_n);
        } else if (!voltage_mode_ && angle_center.has_value()) {
            path = sine
                ? make_sine_path(
                      angle_center->x, angle_center->y, config_.scan_width_deg,
                      config_.scan_height_deg, config_.scan_sine_cycles, 128)
                : make_rectangle_snake_path(
                      angle_center->x, angle_center->y, config_.scan_width_deg,
                      config_.scan_height_deg, config_.scan_grid_n);
        }

        if (path.empty()) {
            continue;
        }
        const std::string error = run_pass(path);
        if (!error.empty()) {
            std::println(stderr, "guidance: {}", error);
        }
    }
}

} // namespace rmcs_laser_guidance
