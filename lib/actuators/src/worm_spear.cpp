#include <Arduino.h>
#include "actuators/worm_spear.hpp"
#include "FastAccelStepper.h"
#include "drivers/servo.hpp"

using control::WormSpear;

static constexpr char TAG[] = "worm_spear";

WormSpear::WormSpear(const Config &config)
    : _config(config)
    , _spear_servo{_config.spear_servo_config}
    , _worm_limit_switch{_config.worm_calibration_switch_pin, INPUT_PULLUP, HIGH}
{
}

esp_err_t WormSpear::init()
{
    configASSERT(_config.engine != nullptr);
    configASSERT(_config.worm_step_pin != GPIO_NUM_NC &&
                 _config.worm_calibration_switch_pin != GPIO_NUM_NC &&
                 _config.worm_dir_pin != GPIO_NUM_NC);

    _stepper = _config.engine->stepperConnectToPin(_config.worm_step_pin, _config.driver_type);

    if (_stepper == nullptr) return ESP_ERR_INVALID_ARG;

    _stepper->setDirectionPin(_config.worm_dir_pin);
    _stepper->setEnablePin(_config.worm_en_pin);
    _stepper->setAutoEnable(true);

    _stepper->setSpeedInHz(_config.speed_hz);
    _stepper->setAcceleration(_config.acceleration_hz_per_s);

    pinMode(_config.worm_calibration_switch_pin, INPUT_PULLUP);
    if (!_spear_servo.init()) return ESP_ERR_INVALID_STATE;
    _spear_servo.set_deg(0.0f);

    _worm_limit_switch.register_pressed_callback(
        [](void *ctx) {
            auto *worm = static_cast<control::WormSpear *>(ctx);
            worm->stop_worm();
        },
        this);

    if (!_worm_limit_switch.begin("worm_spear_limit")) {
        _spear_servo.detach();
        _stepper->disableOutputs();
        return ESP_ERR_NO_MEM;
    }

    return ESP_OK;
}

esp_err_t WormSpear::calibrate()
{
    if (!_worm_limit_switch.is_pressed()) {
        // move quickly to switch
        _stepper->setSpeedInHz(18000);
        if (_config.reversed) {
            _stepper->runForward();
        } else {
            _stepper->runBackward();
        }

        if (!_worm_limit_switch.wait_until_pressed(_config.calibration_max_delay)) {
            _stepper->forceStop();
            ESP_LOGE(TAG, "timed out during fast calibration approach");
            return ESP_FAIL;
        }
    }

    // back up
    _stepper->forceStopAndNewPosition(0);
    vTaskDelay(pdMS_TO_TICKS(10));

    _stepper->move(!_config.reversed ? 2000 : -2000, true);

    // move slowly to switch
    _stepper->setSpeedInHz(900);
    if (_config.reversed) {
        _stepper->runForward();
    } else {
        _stepper->runBackward();
    }

    if (!_worm_limit_switch.wait_until_pressed(_config.calibration_max_delay)) {
        _stepper->forceStop();
        ESP_LOGE(TAG, "timed out during slow calibration approach");
        return ESP_FAIL;
    }

    _stepper->forceStopAndNewPosition(0);
    _stepper->setSpeedInHz(_config.speed_hz);
    return ESP_OK;
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
