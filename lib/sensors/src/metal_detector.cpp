#include <Arduino.h>
#include "esp32-hal-gpio.h"
#include "esp_log.h"

#include "esp32-hal-adc.h"
#include "esp_check.h"
#include "esp_err.h"

#include "sensors/metal_detector.hpp"

static constexpr char TAG[] = "metal_detector";

namespace metal_detector {
esp_err_t MetalDetector::init()
{
    // quick runtime sanity checks on config
    ESP_RETURN_ON_FALSE(_cfg.gpio_md_pulse != GPIO_NUM_NC && _cfg.gpio_md_adc_in != GPIO_NUM_NC,
                        ESP_ERR_INVALID_ARG,
                        TAG,
                        "invalid GPIO configuration");

    ESP_RETURN_ON_FALSE(_cfg.md_pulse_us > 0 && _cfg.md_blank_us > 0,
                        ESP_ERR_INVALID_ARG,
                        TAG,
                        "invalid metal detector timing");

    pinMode(_cfg.gpio_md_adc_in, INPUT);
    pinMode(_cfg.gpio_md_pulse, OUTPUT);

    digitalWrite(_cfg.gpio_md_pulse, LOW);

    analogReadResolution(12);
    analogSetPinAttenuation(_cfg.gpio_md_adc_in, ADC_6db); // up to 2.2V

    _initialized = true;

    return ESP_OK;
}

esp_err_t MetalDetector::pulse_and_sample()
{
    ESP_RETURN_ON_FALSE(_initialized, ESP_ERR_INVALID_STATE, TAG, "not initialized");

    digitalWrite(_cfg.gpio_md_pulse, HIGH);
    esp_rom_delay_us(_cfg.md_pulse_us);

    digitalWrite(_cfg.gpio_md_pulse, LOW);
    esp_rom_delay_us(_cfg.md_blank_us);

    _raw = analogRead(_cfg.gpio_md_adc_in);

    return ESP_OK;
}

esp_err_t MetalDetector::update()
{
    ESP_RETURN_ON_FALSE(_initialized, ESP_ERR_INVALID_STATE, TAG, "not initialized");

    _update_baseline();

    const float adc_shifted = _raw - _pulsed_baseline;

    if (!_adc_filter_initialized) {
        _sensor_out = adc_shifted;
        _adc_filter_initialized = true;
    } else {
        _sensor_out = _cfg.output_alpha * adc_shifted + (1.0f - _cfg.output_alpha) * _sensor_out;
    }

    _update_metal_state();
    _publish_snapshot();

    if (_cfg.logging_enabled) {
        if (!_baseline_ready) {
            Serial.printf(">calibrating:%d\n>raw:%d\n>pulsed_baseline:%.3f\n",
                          _baseline_count,
                          _raw,
                          _pulsed_baseline);
        } else {
            Serial.printf(">md:%.3f\n>shifted:%.3f\n>raw:%d\n>pulsed_baseline:%.3f\n",
                          _sensor_out,
                          adc_shifted,
                          _raw,
                          _pulsed_baseline);
        }
    }

    return ESP_OK;
}

void MetalDetector::_update_baseline()
{
    if (!_baseline_ready) {
        _baseline_count++;

        if (_baseline_count <= _cfg.baseline_discard) {
            return;
        }

        _baseline_sum += _raw;

        const int nsamples = _baseline_count - _cfg.baseline_discard;
        _pulsed_baseline = _baseline_sum / (float)nsamples;

        if (_baseline_count >= _cfg.baseline_samples) {
            _baseline_ready = true;

            ESP_LOGI(TAG, "metal detector calibration complete: baseline:%.3f\n", _pulsed_baseline);
        }
        return;
    } else {
        _pulsed_baseline =
            _cfg.baseline_alpha * _raw + (1.0f - _cfg.baseline_alpha) * _pulsed_baseline;
        return;
    }
}

void MetalDetector::_update_metal_state()
{
    if (!_baseline_ready) {
        _state = MetalState::METAL_CALIBRATION;
        _detect_count = 0;
        _clear_count = 0;
        return;
    }

    if (_state == MetalState::METAL_CALIBRATION) {
        // just finished calibration, reset state to METAL_NONE
        _state = MetalState::METAL_NONE;
        _detect_count = 0;
        _clear_count = 0;
    }

    switch (_state) {
        case MetalState::METAL_DETECTED: {
            bool should_clear = _cfg.clear_threshold > 0.0f ? _sensor_out < _cfg.clear_threshold
                                                            : _sensor_out > _cfg.clear_threshold;

            if (should_clear) {
                _clear_count++;

                if (_clear_count >= _cfg.clear_count_required) {
                    _state = MetalState::METAL_NONE;
                    _detect_count = 0;
                    _clear_count = 0;

                    ESP_LOGI(TAG, "metal_cleared");
                }
            } else {
                _clear_count = 0;
            }
            break;
        }
        case MetalState::METAL_NONE: {
            bool should_detect = _cfg.detect_threshold > 0.0f ? _sensor_out > _cfg.detect_threshold
                                                              : _sensor_out < _cfg.detect_threshold;

            if (should_detect) {
                _detect_count++;

                if (_detect_count >= _cfg.detect_count_required) {
                    _state = MetalState::METAL_DETECTED;
                    _detect_count = 0;
                    _clear_count = 0;

                    ESP_LOGI(TAG, "metal_detected");
                }
            } else {
                _detect_count = 0;
            }
            break;
        }
        default:
            break;
    }
}

} // namespace metal_detector
