#include <Arduino.h>
#include "control/worm_spear.hpp"
#include "FastAccelStepper.h"
#include "actuators/servo.hpp"

using control::WormSpear;

WormSpear::WormSpear(const Config &config)
    : _config(config)
{
}

esp_err_t WormSpear::init()
{
    if (_config.engine == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }

    _stepper = _config.engine->stepperConnectToPin(_config.worm_step_pin);
    if (_stepper == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }

    _stepper->setDirectionPin(_config.worm_dir_pin);

    _stepper->setSpeedInHz(_config.speed_hz);
    _stepper->setAcceleration(_config.acceleration_hz_per_s);
    pinMode(_config.worm_calibration_switch_pin, INPUT_PULLUP);
    if (!_spear_servo.init()) {
        return ESP_ERR_INVALID_STATE;
    }
    return ESP_OK;
}

void WormSpear::calibrate() {
    // move quickly to switch
    _stepper->setSpeedInHz(2000);
    if (!_config.reversed) {
        _stepper->runForward();
    } else {
        _stepper->runBackward();
    }

    // debounce
    int stableCount = 0;
    while (stableCount < 5) {
        if (digitalRead(_config.worm_calibration_switch_pin) == switch_presed) { 
            stableCount++;
        } else {
            stableCount = 0; // Reset if it was just a noise spike
        }
        vTaskDelay(1);
    }
    
    // back up
    _stepper->forceStopAndNewPosition(0);
    vTaskDelay(pdMS_TO_TICKS(10)); 
    _stepper->move(_config.reversed ? 200 : -200); 
    
    while (_stepper->isRunning()) {
        vTaskDelay(1); 
    }

    // move slowly to switch
    _stepper->setSpeedInHz(500);
    if (!_config.reversed) {
        _stepper->runForward();
    } else {
        _stepper->runBackward();
    }

    stableCount = 0;
    while (stableCount < 5) {
        if (digitalRead(_config.worm_calibration_switch_pin) == switch_presed) {
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

void WormSpear::move_to_position(float step) {
    step = _config.reversed ? -step : step;
    _stepper->moveTo(step);
}

void WormSpear::spear_angle(float deg) {
    _spear_servo.set_deg(deg);
}

bool WormSpear::worm_done() const {
    return !_stepper->isRunning();
}

// ONLY FOR TUNING PURPOSES
void WormSpear::set_speed(int32_t speed_hz, unsigned int acceleration_hz_per_s) {
    _stepper->setSpeedInHz(speed_hz);
    _stepper->setAcceleration(acceleration_hz_per_s);
}

void WormSpear::set_home_position() {
    _stepper->setCurrentPosition(0);
}

int32_t WormSpear::get_current_position() {
    int32_t pos = _stepper->getCurrentPosition();
    return _config.reversed ? -pos : pos;
}