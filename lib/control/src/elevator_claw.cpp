#include <Arduino.h>
#include "control/elevator_arm.hpp"
#include "actuators/servo.hpp"
#include "FastAccelStepper.h"

using control::ElevatorClaw;

ElevatorClaw::ElevatorClaw(const Config &config)
    : _config(config)
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
    _arm_servo.init();
    return ESP_OK;
}

void ElevatorClaw::calibrate() {
    return;
}

void ElevatorClaw::move_to_position(float step) {
    _stepper->moveTo(step);
}

void ElevatorClaw::open_claw_tower() {
    _arm_servo.set_deg(180.0f);
    return;
}

void ElevatorClaw::open_claw_rock() {
    _arm_servo.set_deg(180.0f);
    return;
}

void ElevatorClaw::close_arm() {
    _arm_servo.set_deg(0.0f);
    return;
}

bool ElevatorClaw::elevator_done() const {
    return !_stepper->isRunning();
}