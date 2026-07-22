#include "control/pid.hpp"

namespace control {
class TapePID : public PID {
  public:
    explicit TapePID(float kp, float ki, float kd, float d_alpha = 0.0f)
        : PID(kp, ki, kd), prev_derivative_reference(prev_derivative_reference), prev_derivative_reference_time(0) {}

    float update(float ref, float meas, float dt_s) override;

    protected:
    float prev_derivative_reference = 0.0f; // Low-pass filter coefficient for derivative term
    unsigned long prev_derivative_reference_time = 0; // Time of the last derivative reference update
};

}// namespace control