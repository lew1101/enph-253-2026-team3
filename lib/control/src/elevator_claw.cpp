#include <Arduino.h>
#include "control/elevator_claw.hpp"
#include "actuators/servo.hpp"
#include "FastAccelStepper.h"

using control::ElevatorClaw;

ElevatorClaw::ElevatorClaw(const Config &config)
    : _config{config},
    _claw_servo{_config.claw_servo_config},

{
}

esp_err_t ElevatorClaw::init()
{
    if (_config.engine == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }

    _stepper = _config.engine->stepperConnectToPin(_config.elevator_step_pin);
    if (_stepper == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }

    _stepper->setDirectionPin(_config.elevator_dir_pin);

    _stepper->setSpeedInHz(_config.speed_hz);
    _stepper->setAcceleration(_config.acceleration_hz_per_s);
    _stepper->setCurrentPosition(0);
    _claw_servo.init();
    return ESP_OK;
}

void ElevatorClaw::calibrate() {
    return;
}

void ElevatorClaw::move_to_position(int32_t step) {
    _stepper->moveTo(step);
}

void ElevatorClaw::set_claw(float deg) {
    _claw_servo.set_deg(deg);
}

void ElevatorClaw::stop_elevator() {
    _stepper->forceStop();
}

bool ElevatorClaw::elevator_done() const {
    return !_stepper->isRunning();
}
