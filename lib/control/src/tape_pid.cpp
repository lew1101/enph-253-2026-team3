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
        float prev_derivative_reference = _prev_err;
        float prev_derivative_reference_time = millis();
    }

    float derivative = _kd * (error - prev_derivative_reference) / (millis() - prev_derivative_reference_time) * 1000.0f; // Convert to seconds

    // candidate integral update
    float candidate_integral = clamp(_integral + error * dt_s, -_integral_limit, _integral_limit);

    float candidate_output = proportional + _ki * candidate_integral + derivative;
    float output = clamp(candidate_output, _min_output, _max_output);
    if (error != _prev_err) {
        ESP_LOGI("TapePID", "Error: %.2f, Proportional: %.2f, Integral: %.2f, Derivative: %.2f, Output: %.2f", error, proportional, _ki * candidate_integral, derivative, candidate_output);
    }

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
