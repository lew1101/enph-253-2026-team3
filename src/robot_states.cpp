#pragma once
#include <atomic>
#include <cstdint>
#include "robot_states.hpp"

namespace state {
    extern std::atomic<float> g_tape_error{0.0f};
}