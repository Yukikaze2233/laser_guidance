#include "guidance/scan_path.hpp"

#include <algorithm>
#include <cmath>

namespace rmcs_laser_guidance {

auto make_rectangle_snake_path(float cx, float cy, float width, float height, int rows)
    -> std::vector<std::pair<float, float>> {
    const int n = std::max(2, rows);
    const float hw = width * 0.5F;
    const float hh = height * 0.5F;
    const float step = height / static_cast<float>(n - 1);

    std::vector<std::pair<float, float>> pts;
    pts.reserve(static_cast<std::size_t>(2 * n));
    for (int row = 0; row < n; ++row) {
        const float y = cy - hh + step * static_cast<float>(row);
        if (row % 2 == 0) {
            pts.emplace_back(cx - hw, y);
            pts.emplace_back(cx + hw, y);
        } else {
            pts.emplace_back(cx + hw, y);
            pts.emplace_back(cx - hw, y);
        }
    }
    return pts;
}

auto make_sine_path(float cx, float cy, float width, float height, int cycles, int samples)
    -> std::vector<std::pair<float, float>> {
    const int n = std::max(2, samples);
    const float hw = width * 0.5F;
    const float hh = height * 0.5F;
    const float two_pi = 2.0F * 3.14159265358979323846F;

    std::vector<std::pair<float, float>> pts;
    pts.reserve(static_cast<std::size_t>(n));
    for (int i = 0; i < n; ++i) {
        const float frac = static_cast<float>(i) / static_cast<float>(n - 1);
        const float x = cx - hw + width * frac;
        const float y = cy + hh * std::sin(two_pi * static_cast<float>(cycles) * frac);
        pts.emplace_back(x, y);
    }
    return pts;
}

} // namespace rmcs_laser_guidance
