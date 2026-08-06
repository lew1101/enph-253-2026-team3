#include <Arduino.h>
#include "actuators/elevator_claw.hpp"
#include "FastAccelStepperEngine.h"
#include "drivers/servo.hpp"
#include "FastAccelStepper.h"

using control::ElevatorClaw;

static constexpr char TAG[] = "elevator_claw";

ElevatorClaw::ElevatorClaw(const Config &config)
    : _config{config}
    , _claw_servo{_config.claw_servo_config}
    , _limit_switch{_config.elevator_calibration_switch_pin, INPUT_PULLUP, HIGH}
{
}

esp_err_t ElevatorClaw::init()
{
    if (_config.engine == nullptr) return ESP_ERR_INVALID_ARG;

    _stepper = _config.engine->stepperConnectToPin(_config.elevator_step_pin);
    if (_stepper == nullptr) return ESP_ERR_INVALID_ARG;

    _stepper->setDirectionPin(_config.elevator_dir_pin);
    _stepper->setEnablePin(_config.elevator_en_pin);

    _stepper->setDelayToEnable(1000); // µs before first step
    _stepper->setDelayToDisable(1);   // ms after stopping

    _stepper->setSpeedInHz(_config.speed_hz);
    _stepper->setAcceleration(_config.acceleration_hz_per_s);
    _stepper->setCurrentPosition(0);

    if (!_claw_servo.init()) {
        return ESP_ERR_INVALID_STATE;
    }

    _limit_switch.register_pressed_callback(
        [](void *ctx) {
            auto *elevator = static_cast<ElevatorClaw *>(ctx);
            elevator->stop_elevator();
        },
        this);

    if (!_limit_switch.begin("elevator_claw_limit")) {
        _claw_servo.detach();
        _stepper->disableOutputs();
        return ESP_ERR_NO_MEM;
    }

    return ESP_OK;
}

esp_err_t ElevatorClaw::calibrate()
{
    if (!_limit_switch.is_pressed()) {
        // move quickly to switch
        _stepper->setSpeedInHz(2000);
        if (_config.reversed) {
            _stepper->runForward();
        } else {
            _stepper->runBackward();
        }

        if (!_limit_switch.wait_until_pressed(_config.calibration_max_delay)) {
            _stepper->forceStop();
            ESP_LOGE(TAG, "timed out during fast calibration approach");
            return ESP_FAIL;
        }
    }

    // back up
    _stepper->forceStopAndNewPosition(0);
    vTaskDelay(pdMS_TO_TICKS(10));

    _stepper->move(!_config.reversed ? 125 : -125, true);

    // move slowly to switch
    _stepper->setSpeedInHz(500);
    if (_config.reversed) {
        _stepper->runForward();
    } else {
        _stepper->runBackward();
    }

    if (!_limit_switch.wait_until_pressed(_config.calibration_max_delay)) {
        _stepper->forceStop();
        ESP_LOGE(TAG, "timed out during slow calibration approach");
        return ESP_FAIL;
    }

    _stepper->forceStopAndNewPosition(0);
    _stepper->setSpeedInHz(_config.speed_hz);
    return ESP_OK;
}

// ONLY FOR TUNING PURPOSES
void ElevatorClaw::set_speed(int32_t speed_hz, unsigned int acceleration_hz_per_s)
{
    _stepper->setSpeedInHz(speed_hz);
    _stepper->setAcceleration(acceleration_hz_per_s);
}

void ElevatorClaw::set_home_position() { _stepper->setCurrentPosition(0); }

int32_t ElevatorClaw::get_current_position()
{
    int32_t pos = _stepper->getCurrentPosition();
    return _config.reversed ? -pos : pos;
}
