#include <Arduino.h>
#include "control/elevator_claw.hpp"
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
    _stepper->setCurrentPosition(0);
    pinMode(_config.elevator_calibration_switch_pin, INPUT_PULLUP);
    _claw_servo.init();
    return ESP_OK;
}

void ElevatorClaw::calibrate() {

    // move quickly to switch
    _stepper->setSpeedInHz(2000);
    if (_config.direction == 1) {
        _stepper->runForward();
    } else {
        _stepper->runBackward();
    }

    // debounce
    int stableCount = 0;
    while (stableCount < 5) {
        if (digitalRead(_config.elevator_calibration_switch_pin) == switch_presed) { 
            stableCount++;
        } else {
            stableCount = 0; // Reset if it was just a noise spike
        }
        vTaskDelay(1);
    }
    
    // back up
    _stepper->forceStopAndNewPosition(0);
    vTaskDelay(pdMS_TO_TICKS(10)); 
    _stepper->move(-200 * _config.direction); 
    
    while (_stepper->isRunning()) {
        vTaskDelay(1); 
    }

    // move slowly to switch
    _stepper->setSpeedInHz(500);
    if (_config.direction == 1) {
        _stepper->runForward();
    } else {
        _stepper->runBackward();
    }

    stableCount = 0;
    while (stableCount < 5) {
        if (digitalRead(_config.elevator_calibration_switch_pin) == switch_presed) {
            stableCount++;
        } else {
            stableCount = 0;
        }
        vTaskDelay(1);
    }
    
    _stepper->forceStopAndNewPosition(0);
    _stepper->setSpeedInHz(_config.speed_hz);
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

// ONLY FOR TUNING PURPOSES
void ElevatorClaw::set_speed(int32_t speed_hz, unsigned int acceleration_hz_per_s) {
    _stepper->setSpeedInHz(speed_hz);
    _stepper->setAcceleration(acceleration_hz_per_s);
}

void ElevatorClaw::set_home_position() {
    _stepper->setCurrentPosition(0);
}

int32_t ElevatorClaw::get_current_position() {
    return _stepper->getCurrentPosition();
}