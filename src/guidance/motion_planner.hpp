#pragma once

#include <deque>
#include <optional>
#include <utility>

namespace rmcs_laser_guidance {

class MotionPlanner {
public:
    MotionPlanner(float max_velocity, float acceleration, float sample_period_s) noexcept;

    void set_origin(float x, float y) noexcept;
    void move_to(float x, float y) noexcept;
    auto tick(float dt_s) -> std::optional<std::pair<float, float>>;
    [[nodiscard]] auto done() const noexcept -> bool;
    void reset() noexcept;

private:
    struct Segment {
        float x0, y0, x1, y1;
        float length;
        float v_peak;
        float t_accel;
        float t_total;
    };

    float max_velocity_;
    float acceleration_;
    float sample_period_s_;
    std::deque<Segment> segments_{};
    float cursor_x_ = 0.0F;
    float cursor_y_ = 0.0F;
    float t_ = 0.0F;
};

} // namespace rmcs_laser_guidance
