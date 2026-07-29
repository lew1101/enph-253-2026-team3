#include <Arduino.h>
#include "control/worm_spear.hpp"
#include "FastAccelStepper.h"
#include "actuators/servo.hpp"

using control::WormSpear;

static constexpr char TAG[] = "worm_spear";

WormSpear::WormSpear(const Config &config)
    : _config(config)
    , _spear_servo{_config.spear_servo_config}
    , _limit_switch{_config.worm_calibration_switch_pin, INPUT_PULLUP, HIGH}
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

    _limit_switch.register_pressed_callback(
        [](void *ctx) {
            auto *worm = static_cast<control::WormSpear *>(ctx);
            worm->stop_worm();
        },
        this);

    if (!_limit_switch.begin("worm_spear_limit")) {
        return ESP_ERR_NO_MEM;
    }

    return ESP_OK;
}

void WormSpear::calibrate()
{
    // move quickly to switch
    _stepper->setSpeedInHz(2000);
    if (!_config.reversed) {
        _stepper->runForward();
    } else {
        _stepper->runBackward();
    }

    if (!_limit_switch.wait_until_pressed(_config.calibration_max_delay)) {
        _stepper->forceStop();
        ESP_LOGE(TAG, "timed out during fast calibration approach");
        return;
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

    if (!_limit_switch.wait_until_pressed(_config.calibration_max_delay)) {
        _stepper->forceStop();
        ESP_LOGE(TAG, "timed out during slow calibration approach");
        return;
    }

    _stepper->forceStopAndNewPosition(0);
    _stepper->setSpeedInHz(_config.speed_hz);
    return;
}

// ONLY FOR TUNING PURPOSES
void WormSpear::set_speed(int32_t speed_hz, unsigned int acceleration_hz_per_s)
{
    _stepper->setSpeedInHz(speed_hz);
    _stepper->setAcceleration(acceleration_hz_per_s);
}

void WormSpear::set_home_position() { _stepper->setCurrentPosition(0); }

int32_t WormSpear::get_current_position()
{
    int32_t pos = _stepper->getCurrentPosition();
    return _config.reversed ? -pos : pos;
}
