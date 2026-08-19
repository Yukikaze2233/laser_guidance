#include <cmath>
#include <cstdlib>
#include <print>
#include <utility>

#include "guidance/motion_planner.hpp"

namespace {
using namespace rmcs_laser_guidance;

#define CHECK(cond)                         \
    do {                                    \
        if (!(cond)) {                      \
            std::println("  FAIL: " #cond); \
            std::abort();                   \
        }                                   \
    } while (0)

void test_accel_phase() {
    // v_max=10, a=100, L=10 -> trapezoid: t_a=0.1, cruise 9@0.9s, T=1.1s
    MotionPlanner planner(10.0F, 100.0F, 0.001F);
    planner.set_origin(0.0F, 0.0F);
    planner.move_to(10.0F, 0.0F);
    const auto p = planner.tick(0.001F);
    CHECK(p.has_value());
    // s(t) = 0.5*a*t^2 at t=0.001 -> 5e-5
    CHECK(std::fabs(p->first - 5e-5F) < 1e-8F);
    CHECK(std::fabs(p->second) < 1e-6F);
    std::println("  accel_phase: OK");
}

void test_uniform_cruise() {
    MotionPlanner planner(10.0F, 100.0F, 0.001F);
    planner.set_origin(0.0F, 0.0F);
    planner.move_to(10.0F, 0.0F);
    std::optional<std::pair<float, float>> prev;
    for (int i = 0; i < 900; ++i) {   // ticks 101..1000 are cruise (t in [0.1, 1.0])
        const auto p = planner.tick(0.001F);
        CHECK(p.has_value());
        if (i >= 200) {
            if (prev) {
                // cruise speed = v_max=10 -> 0.01 per 1ms tick
                CHECK(std::fabs((p->first - prev->first) - 0.01F) < 1e-4F);
            }
            prev = p;
        }
    }
    std::println("  uniform_cruise: OK");
}

void test_triangle_profile() {
    // v_max=10, a=100, L=0.5: L*a=50 < 100 -> triangle, v_peak=sqrt(50)~7.0711, T~0.1414
    MotionPlanner planner(10.0F, 100.0F, 0.001F);
    planner.set_origin(0.0F, 0.0F);
    planner.move_to(0.5F, 0.0F);
    for (int i = 0; i < 69; ++i) {
        CHECK(planner.tick(0.001F).has_value());
    }
    const auto p70 = planner.tick(0.001F);   // t=0.070: s = 0.5*a*t^2 = 0.245
    CHECK(p70.has_value());
    CHECK(std::fabs(p70->first - 0.245F) < 1e-3F);
    int remaining = 0;
    while (planner.tick(0.001F).has_value())
        ++remaining;
    CHECK(remaining == 72);                  // 141 total ticks, 70 consumed, 71 left -> 72 with pop
    CHECK(planner.done());
    std::println("  triangle_profile: OK");
}

void test_endpoint_exactness() {
    MotionPlanner planner(10.0F, 100.0F, 0.001F);
    planner.set_origin(0.0F, 0.0F);
    planner.move_to(1.0F, 0.0F);
    planner.move_to(1.0F, 1.0F);
    planner.move_to(0.0F, 1.0F);
    std::optional<std::pair<float, float>> last;
    int ticks = 0;
    while (auto p = planner.tick(0.001F)) {
        last = p;
        ++ticks;
    }
    // each segment L=1 -> T=0.2s -> 200 ticks each, total 600
    CHECK(ticks == 600);
    CHECK(last.has_value());
    CHECK(last->first == 0.0F && last->second == 1.0F);
    CHECK(planner.done());
    std::println("  endpoint_exactness: OK");
}

void test_zero_length_skipped() {
    MotionPlanner planner(10.0F, 100.0F, 0.001F);
    planner.set_origin(0.0F, 0.0F);
    planner.move_to(1.0F, 0.0F);
    planner.move_to(1.0F, 0.0F);   // zero-length, must be skipped
    int ticks = 0;
    while (planner.tick(0.001F).has_value())
        ++ticks;
    CHECK(ticks == 200);           // only the first segment runs
    CHECK(planner.done());
    std::println("  zero_length_skipped: OK");
}

} // namespace

int main() {
    std::println("motion_planner_test");
    test_accel_phase();
    test_uniform_cruise();
    test_triangle_profile();
    test_endpoint_exactness();
    test_zero_length_skipped();
    return 0;
}
