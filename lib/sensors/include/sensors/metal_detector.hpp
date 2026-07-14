#pragma once

#include "soc/gpio_num.h"
#include "portmacro.h"

namespace metal_detector {
enum class MetalState : uint8_t { METAL_CALIBRATION, METAL_NONE, METAL_DETECTED };

class MetalDetector {
  public:
    struct Config {
        gpio_num_t gpio_md_pulse = GPIO_NUM_NC;
        gpio_num_t gpio_md_adc_in = GPIO_NUM_NC;

        uint32_t md_pulse_us = 100;
        uint32_t md_blank_us = 50;

        uint32_t detect_count_required = 4;
        uint32_t clear_count_required = 8;

        float detect_threshold = 7.0f;
        float clear_threshold = 4.0f;

        int baseline_samples = 400;
        int baseline_discard = 70;

        float output_alpha = 0.06f;
        float baseline_alpha = 0.00005f;
    };

    struct Snapshot {
        int raw = 0;
        float sensor = 0.0f;
        float baseline = 0.0f;
        bool baseline_ready = false;
        MetalState state = MetalState::METAL_CALIBRATION;
    };

  private:
    Config _cfg;
    Snapshot _snapshot;
    MetalState _state = MetalState::METAL_CALIBRATION;

    portMUX_TYPE _mux = portMUX_INITIALIZER_UNLOCKED;

    float _pulsed_baseline = 0.0f;
    float _sensor_out = 0.0f;

    bool _adc_filter_initialized = false;
    bool _baseline_ready = false;

    int _baseline_count = 0;
    float _baseline_sum = 0.0f;

    uint32_t _detect_count = 0;
    uint32_t _clear_count = 0;

    bool _initialized = false;
    int _raw;

  public:
    explicit MetalDetector(const Config &cfg)
        : _cfg(cfg)
    {
    }
    ~MetalDetector() = default;

    MetalDetector(const MetalDetector &other) = delete;
    MetalDetector &operator=(const MetalDetector &other) = delete;

    esp_err_t init();
    inline bool is_calibration_complete() {return _baseline_ready;}

    esp_err_t pulse_and_sample();
    esp_err_t update();

    inline void get_snapshot(Snapshot &out)
    {
        portENTER_CRITICAL(&_mux);
        out = _snapshot;
        portEXIT_CRITICAL(&_mux);
    }

  private:
    void _update_baseline();
    void _update_metal_state();

    inline void _publish_snapshot()
    {
        portENTER_CRITICAL(&_mux); // prevent interrupts from chaning snapshot
        _snapshot.raw = _raw;
        _snapshot.sensor = _sensor_out;
        _snapshot.baseline = _pulsed_baseline;
        _snapshot.baseline_ready = _baseline_ready;
        _snapshot.state = _state;
        portEXIT_CRITICAL(&_mux);
    }
};
} // namespace metal_detector
