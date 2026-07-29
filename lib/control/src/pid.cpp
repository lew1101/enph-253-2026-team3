#include <Arduino.h>
#include <algorithm>

#include "control/pid.hpp"

static constexpr char TAG[]{"pid"};

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
    float candidate_integral = _integral + error * dt_s;
    float integral = clamp(candidate_integral, -_integral_limit, _integral_limit);

    float candidate_output = proportional + _ki * integral + derivative;
    float output = clamp(candidate_output, _min_output, _max_output);

    if (candidate_integral != integral) {
        ESP_LOGD(TAG, "integral clamped");
    }

    /*
     * anti-windup:
     * accept the new integral when:
     * - the actuator is not saturated; or
     * - it is saturated high and the current error reduces the integral; or
     * - it is saturated low and the current error increases the integral.
     */
    bool saturated_high = candidate_output > _max_output;
    bool saturated_low = candidate_output < _min_output;

    if (saturated_high) {
        // ESP_LOGD(TAG, "saturated high");
    } else if (saturated_low) {
        // ESP_LOGD(TAG, "saturated low");
    }

    bool drives_out_of_high_saturation = saturated_high && error < 0.0f;
    bool drives_out_of_low_saturation = saturated_low && error > 0.0f;

    if ((!saturated_high && !saturated_low) || drives_out_of_high_saturation ||
        drives_out_of_low_saturation) {
        _integral = integral;
    } else {
        // ESP_LOGD(TAG, "anti-windup activated");
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
