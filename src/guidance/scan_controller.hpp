#pragma once

#include <condition_variable>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

#include <opencv2/core/types.hpp>

#include "config.hpp"
#include "guidance/motion_planner.hpp"
#include "guidance/scan_path.hpp"

namespace rmcs_laser_guidance {

class GalvoExecutor;
class GalvoDriver;

class ScanController {
public:
    ScanController(const GuidanceConfig& config, GalvoExecutor& executor);
    ~ScanController();

    ScanController(const ScanController&) = delete;
    auto operator=(const ScanController&) -> ScanController& = delete;

    [[nodiscard]] auto enabled() const noexcept -> bool { return enabled_; }
    auto update_angles_center(const cv::Point2f& center) -> void;
    auto update_voltage_center(const cv::Point2f& center) -> void;
    auto deactivate() -> void;
    auto stop() -> void;

private:
    auto run_pass(const std::vector<std::pair<float, float>>& path) -> std::string;
    auto run() -> void;

    GuidanceConfig config_;
    GalvoExecutor& executor_;
    bool enabled_ = false;
    bool voltage_mode_ = false;
    std::mutex mutex_;
    std::condition_variable cv_;
    std::jthread worker_;
    bool stop_requested_ = false;
    bool active_ = false;
    std::optional<cv::Point2f> angle_center_{};
    std::optional<cv::Point2f> voltage_center_{};
};

} // namespace rmcs_laser_guidance
