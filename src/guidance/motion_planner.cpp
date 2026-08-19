#include "guidance/motion_planner.hpp"

#include <cmath>

namespace rmcs_laser_guidance {

MotionPlanner::MotionPlanner(float max_velocity, float acceleration, float sample_period_s) noexcept
    : max_velocity_(max_velocity)
    , acceleration_(acceleration)
    , sample_period_s_(sample_period_s) {}

void MotionPlanner::set_origin(float x, float y) noexcept {
    cursor_x_ = x;
    cursor_y_ = y;
}

void MotionPlanner::move_to(float x, float y) noexcept {
    const float dx = x - cursor_x_;
    const float dy = y - cursor_y_;
    const float length = std::hypot(dx, dy);
    if (length <= 1e-6F)
        return;

    const float v_max2 = max_velocity_ * max_velocity_;
    const float v_peak =
        (length * acceleration_ >= v_max2) ? max_velocity_ : std::sqrt(acceleration_ * length);
    const float t_accel = v_peak / acceleration_;
    const float t_total = 2.0F * t_accel + (length - v_peak * v_peak / acceleration_) / v_peak;

    segments_.push_back({cursor_x_, cursor_y_, x, y, length, v_peak, t_accel, t_total});
    cursor_x_ = x;
    cursor_y_ = y;
}

auto MotionPlanner::tick(float dt_s) -> std::optional<std::pair<float, float>> {
    if (segments_.empty())
        return std::nullopt;

    auto& seg = segments_.front();
    t_ += dt_s;
    if (t_ >= seg.t_total) {
        const auto result = std::pair(seg.x1, seg.y1);
        segments_.pop_front();
        t_ = 0.0F;
        return result;
    }

    const float t = t_;
    const float v = seg.v_peak;
    const float a = acceleration_;
    const float ta = seg.t_accel;
    float s;
    if (t <= ta) {
        s = 0.5F * a * t * t;
    } else if (t <= seg.t_total - ta) {
        s = v * (t - ta) + 0.5F * a * ta * ta;
    } else {
        const float rem = seg.t_total - t;
        s = seg.length - 0.5F * a * rem * rem;
    }

    const float frac = s / seg.length;
    return std::pair(seg.x0 + (seg.x1 - seg.x0) * frac, seg.y0 + (seg.y1 - seg.y0) * frac);
}

auto MotionPlanner::done() const noexcept -> bool { return segments_.empty(); }

void MotionPlanner::reset() noexcept {
    segments_.clear();
    t_ = 0.0F;
}

} // namespace rmcs_laser_guidance
