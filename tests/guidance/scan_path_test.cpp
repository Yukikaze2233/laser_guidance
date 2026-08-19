#include <cmath>
#include <cstdlib>
#include <print>
#include <utility>
#include <vector>

#include "guidance/scan_path.hpp"

namespace {
using namespace rmcs_laser_guidance;

#define CHECK(cond)                         \
    do {                                    \
        if (!(cond)) {                      \
            std::println("  FAIL: " #cond); \
            std::abort();                   \
        }                                   \
    } while (0)

void test_rectangle_snake() {
    const auto pts = make_rectangle_snake_path(0.0F, 0.0F, 10.0F, 8.0F, 4);
    CHECK(pts.size() == 8);
    // row 0: left -> right at y=-4
    CHECK(pts[0].first == -5.0F && pts[0].second == -4.0F);
    CHECK(pts[1].first == 5.0F && pts[1].second == -4.0F);
    // row 1: right -> left at y=-4+8/3
    const float y1 = -4.0F + 8.0F / 3.0F;
    CHECK(std::fabs(pts[2].second - y1) < 1e-4F);
    CHECK(pts[2].first == 5.0F);
    CHECK(pts[3].first == -5.0F);
    // last row: y=4
    CHECK(pts[6].first == 5.0F && std::fabs(pts[6].second - 4.0F) < 1e-4F);
    CHECK(pts[7].first == -5.0F && std::fabs(pts[7].second - 4.0F) < 1e-4F);
    std::println("  rectangle_snake: OK");
}

void test_rectangle_snake_min_rows() {
    const auto pts = make_rectangle_snake_path(0.0F, 0.0F, 10.0F, 8.0F, 1);
    CHECK(pts.size() == 4);   // rows clamped to 2: y=-4 and y=4
    CHECK(pts[0].second == -4.0F);
    CHECK(pts[2].second == 4.0F);
    std::println("  rectangle_snake_min_rows: OK");
}

void test_sine_path() {
    const auto pts = make_sine_path(0.0F, 0.0F, 10.0F, 8.0F, 2, 9);
    CHECK(pts.size() == 9);
    CHECK(pts[0].first == -5.0F && std::fabs(pts[0].second) < 1e-4F);
    // i=1, frac=0.125: x=-3.75, y=4*sin(pi/2)=4
    CHECK(std::fabs(pts[1].first + 3.75F) < 1e-4F);
    CHECK(std::fabs(pts[1].second - 4.0F) < 1e-4F);
    // i=4, frac=0.5: x=0, y=0
    CHECK(std::fabs(pts[4].first) < 1e-4F && std::fabs(pts[4].second) < 1e-4F);
    // i=7, frac=0.875: x=3.75, y=4*sin(3.5*pi)=-4
    CHECK(std::fabs(pts[7].first - 3.75F) < 1e-4F);
    CHECK(std::fabs(pts[7].second + 4.0F) < 1e-4F);
    // last: x=5, y=0
    CHECK(std::fabs(pts[8].first - 5.0F) < 1e-4F && std::fabs(pts[8].second) < 1e-4F);
    std::println("  sine_path: OK");
}

} // namespace

int main() {
    std::println("scan_path_test");
    test_rectangle_snake();
    test_rectangle_snake_min_rows();
    test_sine_path();
    return 0;
}
