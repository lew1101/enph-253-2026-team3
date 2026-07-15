#include <atomic>
#include <cstdint>
#include "robot_states.hpp"

namespace state {
    std::atomic<float> g_tape_error{0.0f};
}