#pragma once
#include "control/pid.hpp"

namespace control {
class TapePID : public PID {
  public:
    explicit TapePID(float kp, float ki, float kd, float d_alpha = 0.0f)
        : PID(kp, ki, kd), _prev_derivative_reference(0.0f), _prev_time_elapse(0.0f) {}

    float update(float ref, float meas, float dt_s) override;

    protected:
    float _prev_derivative_reference = 0.0f; // Low-pass filter coefficient for derivative term
    float _prev_time_elapse = 0.0f; // Time since the last derivative reference update
};

}// namespace control
