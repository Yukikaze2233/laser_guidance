#pragma once

#include <utility>
#include <vector>

namespace rmcs_laser_guidance {

auto make_rectangle_snake_path(float cx, float cy, float width, float height, int rows)
    -> std::vector<std::pair<float, float>>;

auto make_sine_path(float cx, float cy, float width, float height, int cycles, int samples)
    -> std::vector<std::pair<float, float>>;

} // namespace rmcs_laser_guidance
