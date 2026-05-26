#include <Arduino.h>
#include <algorithm>

#include "control/pid.hpp"

namespace control {

float PID::update(float ref, float meas, float dt_s)
{
    using std::clamp;

    float error = ref - meas;

    // do not update time-dependent state with an invalid timestep
    if (dt_s <= 0.0f) return 0.0f;

    float proportional = _kp * error;
    float derivative = _initialized ? _kd * (error - _prev_err) / dt_s : 0.0f;

    // candidate integral update
    float candidate_integral = clamp(_integral + error * dt_s, -_integral_limit, _integral_limit);

    float candidate_output = proportional + _ki * candidate_integral + derivative;
    float output = clamp(candidate_output, _min_output, _max_output);

    /*
     * anti-windup:
     * accept the new integral when:
     * - the actuator is not saturated; or
     * - it is saturated high and the current error reduces the integral; or
     * - it is saturated low and the current error increases the integral.
     */
    bool saturated_high = candidate_output > _max_output;
    bool saturated_low = candidate_output < _min_output;

    bool drives_out_of_high_saturation = saturated_high && error < 0.0f;
    bool drives_out_of_low_saturation = saturated_low && error > 0.0f;

    if ((!saturated_high && !saturated_low) || drives_out_of_high_saturation ||
        drives_out_of_low_saturation) {
        _integral = candidate_integral;
    }

    _prev_err = error;
    _initialized = true;

    return output;
}

void PID::reset()
{
    _integral = 0.0f;
    _prev_err = 0.0f;
    _initialized = false;
}
} // namespace control
