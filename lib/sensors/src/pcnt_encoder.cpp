#include "sensors/pcnt_encoder.hpp"
#include "driver/pulse_cnt.h"

#include "esp_check.h"

static constexpr char TAG[] = "encoder_driver";

namespace sensors {
PcntEncoder::~PcntEncoder() { PcntEncoder::deinit(); }

esp_err_t PcntEncoder::init(const Config &cfg)
{
    ESP_RETURN_ON_FALSE(
        _unit == nullptr, ESP_ERR_INVALID_STATE, TAG, "pcnt unit already initialized");

    _dir_sign = cfg.invert_direction ? -1 : 1;

    pcnt_unit_config_t unit_cfg{};
    unit_cfg.high_limit = kHighLimit;
    unit_cfg.low_limit = kLowLimit;
    unit_cfg.flags.accum_count = true;

    ESP_RETURN_ON_ERROR( //
        pcnt_new_unit(&unit_cfg, &_unit),
        TAG,
        "failed to instantiate pcnt unit");

    esp_err_t err;

    {
        if (cfg.glitch_filter_ns > 0) {
            pcnt_glitch_filter_config_t filter_config{};
            filter_config.max_glitch_ns = cfg.glitch_filter_ns;

            err = pcnt_unit_set_glitch_filter(_unit, &filter_config);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "failed to setup glitch filter");
                goto cleanup;
            }
        }

        /*Channel A : *edge input = A *level input = B */
        pcnt_chan_config_t chan_a_cfg{};
        chan_a_cfg.edge_gpio_num = cfg.gpio_a;
        chan_a_cfg.level_gpio_num = cfg.gpio_b;

        err = pcnt_new_channel(_unit, &chan_a_cfg, &_chan_a);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "failed to setup channel A");
            goto cleanup;
        }

        /*
         * Channel B:
         *   edge input  = B
         *   level input = A
         */
        pcnt_chan_config_t chan_b_cfg{};
        chan_b_cfg.edge_gpio_num = cfg.gpio_b;
        chan_b_cfg.level_gpio_num = cfg.gpio_a;

        err = pcnt_new_channel(_unit, &chan_b_cfg, &_chan_b);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "failed to setup channel B");
            goto cleanup;
        }

        // Configure x4 quadrature decoding.
        err = pcnt_channel_set_edge_action(
            _chan_a, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE);
        if (err != ESP_OK) goto cleanup;

        err = pcnt_channel_set_level_action(
            _chan_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE);
        if (err != ESP_OK) goto cleanup;

        err = pcnt_channel_set_edge_action(
            _chan_b, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE);
        if (err != ESP_OK) goto cleanup;

        err = pcnt_channel_set_level_action(
            _chan_b, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE);
        if (err != ESP_OK) goto cleanup;

        /*
         * Required for accum_count to compensate when the hardware's
         * internal counter reaches its limits.
         */
        err = pcnt_unit_add_watch_point(_unit, kHighLimit);
        if (err != ESP_OK) goto cleanup;

        err = pcnt_unit_add_watch_point(_unit, kLowLimit);
        if (err != ESP_OK) goto cleanup;

        err = pcnt_unit_enable(_unit);
        if (err != ESP_OK) goto cleanup;

        _enabled = true;

        err = pcnt_unit_clear_count(_unit);
        if (err != ESP_OK) goto cleanup;

        err = pcnt_unit_start(_unit);
        if (err != ESP_OK) goto cleanup;

        _initialized = true;

        return ESP_OK;
    }

cleanup:
    deinit();
    return err;
}

esp_err_t PcntEncoder::get_count(int *count) const
{
    ESP_RETURN_ON_FALSE(_unit != nullptr && _initialized,
                        ESP_ERR_INVALID_STATE,
                        TAG,
                        "cannot get count before intialized");

    int raw_count = 0;

    ESP_RETURN_ON_ERROR(pcnt_unit_get_count(_unit, &raw_count), TAG, "failed to get count");

    *count = _dir_sign * raw_count;
    return ESP_OK;
}

esp_err_t PcntEncoder::clear()
{
    ESP_RETURN_ON_FALSE( //
        _unit != nullptr,
        ESP_ERR_INVALID_STATE,
        TAG,
        "cannot clear before intialized");

    return pcnt_unit_clear_count(_unit);
}

esp_err_t PcntEncoder::deinit()
{
    esp_err_t first_err = ESP_OK;
    esp_err_t err;

    if (_unit != nullptr && _initialized) {
        err = pcnt_unit_stop(_unit);
        if (first_err == ESP_OK && err != ESP_OK) {
            first_err = err;
        }
        _initialized = false;
    }

    if (_unit != nullptr && _enabled) {
        err = pcnt_unit_disable(_unit);
        if (first_err == ESP_OK && err != ESP_OK) {
            first_err = err;
        }
        _enabled = false;
    }

    if (_chan_a != nullptr) {
        err = pcnt_del_channel(_chan_a);
        if (first_err == ESP_OK && err != ESP_OK) {
            first_err = err;
        }
        _chan_a = nullptr;
    }

    if (_chan_b != nullptr) {
        err = pcnt_del_channel(_chan_b);
        if (first_err == ESP_OK && err != ESP_OK) {
            first_err = err;
        }
        _chan_b = nullptr;
    }

    if (_unit != nullptr) {
        err = pcnt_del_unit(_unit);
        if (first_err == ESP_OK && err != ESP_OK) {
            first_err = err;
        }
        _unit = nullptr;
    }

    return first_err;
}

} // namespace sensor
