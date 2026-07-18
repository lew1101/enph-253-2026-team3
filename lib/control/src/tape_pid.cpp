#include <algorithm>

#include "control/tape_pid.hpp"

namespace control {

float TapePID::update(float ref, float meas, float dt_s)
{
	using std::clamp;

	float error = ref - meas;

	if (dt_s <= 0.0f) return 0.0f;

	float proportional = _kp * error;

	float raw_derivative = _initialized ? (error - _prev_err) / dt_s : 0.0f;
	_filtered_derivative = _d_alpha * raw_derivative + (1.0f - _d_alpha) * _filtered_derivative;
    float final_derivative = _kd * _filtered_derivative;

    // candidate integral update
    float candidate_integral = clamp(_integral + error * dt_s, -_integral_limit, _integral_limit);

    float candidate_output = proportional + _ki * candidate_integral + final_derivative;
    float output = clamp(candidate_output, _min_output, _max_output);

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

void TapePID::reset()
{
	PID::reset();
	_filtered_derivative = 0.0f;
}

} // namespace control
