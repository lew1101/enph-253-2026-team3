#include <algorithm>

#include "control/tape_pid.hpp"

namespace control {

float TapePID::update(float ref, float meas, float dt_s)
{
	using std::clamp;

	float error = ref - meas;

	if (dt_s <= 0.0f) return 0.0f;

	float proportional = _kp * error;

    if (error != _prev_err) {
        _prev_derivative_reference = _prev_err;
        _prev_time_elapse = 0.0f;
    }
    _prev_time_elapse += dt_s;
    float derivative = _kd * (error - _prev_derivative_reference) / _prev_time_elapse; // Convert to seconds

    // candidate integral update
    float candidate_integral = clamp(_integral + error * dt_s, -_integral_limit, _integral_limit);

    float candidate_output = proportional + _ki * candidate_integral + derivative;
    float output = clamp(candidate_output, _min_output, _max_output);
    // ESP_LOGI("TapePID", "Error: %.2f, Proportional: %.2f, Integral: %.2f, Derivative: %.2f, Output: %.2f, raw dE: %.2f, dds: %.2f", error, proportional, _ki * candidate_integral, derivative, candidate_output, error - _prev_derivative_reference, _prev_time_elapse);
    // ESP_LOGI("TapePID", "Error: %d, Proportional: %.2f, Derivative: %.2f, Output: %.2f", error, proportional, derivative, candidate_output);
    ESP_LOGI("TapePID", "Error: %.2f, Derivative: %.2f", error, derivative);


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

} // namespace control
