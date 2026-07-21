#include "control/pid.hpp"

namespace control {
class TapePID : public PID {
  public:
    explicit TapePID(float kp, float ki, float kd, float d_alpha = 0.2f)
        : PID(kp, ki, kd), _d_alpha(d_alpha), _filtered_derivative(0.0f) {}

    float update(float ref, float meas, float dt_s) override;
    void set_derivative_error (float clamp_error) { _d_clamp_error = clamp_error; }
    void reset() override;

    protected:
    float _d_alpha = 0.1f; // Low-pass filter coefficient for derivative term
    float _filtered_derivative = 0.0f;
    float _d_clamp_error = 5.0f; // Clamp error for derivative term to avoid spikes
};

}// namespace control