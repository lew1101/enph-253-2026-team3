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
    _spear_servo.init();
    return ESP_OK;
}

void WormSpear::calibrate() {
    return;
}

void WormSpear::move_to_position(float step) {
    _stepper->moveTo(step);
}

void WormSpear::spear_angle(float deg) {
    _spear_servo.set_deg(deg);
}

bool WormSpear::worm_done() const {
    return !_stepper->isRunning();
}
