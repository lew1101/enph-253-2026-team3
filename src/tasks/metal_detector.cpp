#include "tasks/metal_detector.hpp"

#include "esp32-hal-gpio.h"
#include "esp32-hal-adc.h"
#include "esp_rom_sys.h"

#include "esp_err.h"
#include "esp_check.h"

#ifdef DEBUG
#define LOG_LOCAL_LEVEL ESP_LOG_VERBOSE
#else
#define LOG_LOCAL_LEVEL ESP_LOG_WARN
#endif

#include "esp32-hal-log.h"

static const char TAG[] = "metal_detector";

namespace metal_detector {
namespace {
// Global state for the metal detector task
MetalConfig s_cfg{};
MetalSnapshot s_snapshot{};

hw_timer_t *s_md_timer = nullptr;
portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;

TaskHandle_t s_task_handle = nullptr;

MetalState s_state = MetalState::METAL_NONE;

float s_pulsed_baseline = 0.0f;
float s_sensor_out = 0.0f;

bool s_adc_filter_initialized = false;
bool s_baseline_ready = false;

int s_baseline_count = 0;
float s_baseline_sum = 0.0f;

uint32_t s_detect_count = 0;
uint32_t s_clear_count = 0;

static inline void IRAM_ATTR arm_timer_us(uint32_t delay_us)
{
    timerWrite(s_md_timer, 0); // reset timer
    timerAlarm(s_md_timer, delay_us, false, 0);
}

static void IRAM_ATTR on_metal_timer()
{
    BaseType_t higher_priority_task_woken = pdFALSE;

    if (s_task_handle != nullptr) {
        vTaskNotifyGiveFromISR(s_task_handle, &higher_priority_task_woken);
    }

    if (higher_priority_task_woken == pdTRUE) {
        portYIELD_FROM_ISR();
    }
}

static void publish_snapshot(int raw)
{
    portENTER_CRITICAL(&s_mux); // prevent interrupts from chaning snapshot

    s_snapshot.raw = raw;
    s_snapshot.sensor = s_sensor_out;
    s_snapshot.baseline = s_pulsed_baseline;
    s_snapshot.baseline_ready = s_baseline_ready;
    s_snapshot.state = s_state;

    portEXIT_CRITICAL(&s_mux);
}

static void update_baseline(float raw)
{
    if (!s_baseline_ready) {
        s_baseline_count++;

        if (s_baseline_count <= s_cfg.baseline_discard) {
            return;
        }

        s_baseline_sum += raw;

        const int nsamples = s_baseline_count - s_cfg.baseline_discard;
        s_pulsed_baseline = s_baseline_sum / (float)nsamples;

        if (s_baseline_count >= s_cfg.baseline_samples) {
            s_baseline_ready = true;

            ESP_LOGI(
                TAG, "metal detector calibration complete: baseline:%.3f\n", s_pulsed_baseline);
        }
        return;
    } else {
        s_pulsed_baseline =
            s_cfg.baseline_alpha * raw + (1.0f - s_cfg.baseline_alpha) * s_pulsed_baseline;
        return;
    }
}

static void update_metal_state()
{
    if (!s_baseline_ready) {
        s_state = MetalState::METAL_CALIBRATION;
        s_detect_count = 0;
        s_clear_count = 0;
        return;
    }

    if (s_state == MetalState::METAL_CALIBRATION) {
        // just finished calibration, reset state to METAL_NONE
        s_state = MetalState::METAL_NONE;
        s_detect_count = 0;
        s_clear_count = 0;
    }

    const float mag = fabsf(s_sensor_out);

    switch (s_state) {
        case MetalState::METAL_DETECTED: {
            if (mag < s_cfg.clear_threshold) {
                s_clear_count++;

                if (s_clear_count >= s_cfg.clear_count_required) {
                    s_state = MetalState::METAL_NONE;
                    s_detect_count = 0;
                    s_clear_count = 0;

                    ESP_LOGI(TAG, "metal_cleared");
                }
            } else {
                s_clear_count = 0;
            }
            break;
        }
        case MetalState::METAL_NONE: {
            if (mag > s_cfg.detect_threshold) {
                s_detect_count++;

                if (s_detect_count >= s_cfg.detect_count_required) {
                    s_state = MetalState::METAL_DETECTED;
                    s_detect_count = 0;
                    s_clear_count = 0;

                    ESP_LOGI(TAG, "metal_detected");
                }
            } else {
                s_detect_count = 0;
            }
            break;
        }
        default:
            break;
    }
}

static void process_sample(int raw)
{
    update_baseline(raw);

    const float adc_shifted = raw - s_pulsed_baseline;

    if (!s_adc_filter_initialized) {
        s_sensor_out = adc_shifted;
        s_adc_filter_initialized = true;
    } else {
        s_sensor_out =
            s_cfg.output_alpha * adc_shifted + (1.0f - s_cfg.output_alpha) * s_sensor_out;
    }

    update_metal_state();
    publish_snapshot(raw);

    if (!s_baseline_ready) {
        ESP_LOGV(TAG,
                 ">calibrating:%d,raw:%d,pulsed_baseline:%.3f\n",
                 s_baseline_count,
                 raw,
                 s_pulsed_baseline);
    } else {
        ESP_LOGV(TAG,
                 ">md:%.3f,shifted:%.3f,raw:%d,pulsed_baseline:%.3f\n",
                 s_sensor_out,
                 adc_shifted,
                 raw,
                 s_pulsed_baseline);
    }
}

static esp_err_t gpio_adc_setup()
{
    pinMode(s_cfg.gpio_md_adc_in, INPUT);
    pinMode(s_cfg.gpio_md_pulse, OUTPUT);

    gpio_set_level(s_cfg.gpio_md_pulse, 0);

    analogReadResolution(12);
    analogSetPinAttenuation(s_cfg.gpio_md_adc_in, ADC_6db); // up to 2.2V

    return ESP_OK;
}

static esp_err_t timer_setup()
{
    // setup timer
    s_md_timer = timerBegin(1000000);

    if (s_md_timer == nullptr) {
        return ESP_FAIL;
    }

    timerAttachInterrupt(s_md_timer, &on_metal_timer); // attach interrupt handler

    arm_timer_us(s_cfg.start_delay_us);

    return ESP_OK;
}

static void metal_task(void *arg)
{
    (void)arg;

    while (true) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        gpio_set_level(s_cfg.gpio_md_pulse, 1);
        esp_rom_delay_us(s_cfg.md_pulse_us);

        gpio_set_level(s_cfg.gpio_md_pulse, 0);
        esp_rom_delay_us(s_cfg.md_blank_us);

        const int raw = analogRead(s_cfg.gpio_md_adc_in);
        process_sample(raw);

        arm_timer_us(s_cfg.md_deadtime_us);
    }
}
} // namespace

esp_err_t start_metal_detector_task(const MetalTaskConfig &task_cfg,
                                    const MetalConfig &detector_cfg,
                                    TaskHandle_t *out_handle)
{
    if (s_task_handle != nullptr) {
        if (out_handle != nullptr) {
            *out_handle = s_task_handle;
        }

        return ESP_ERR_INVALID_STATE;
    }

    // quick runtime sanity checks on config
    ESP_RETURN_ON_FALSE(detector_cfg.gpio_md_pulse != GPIO_NUM_NC &&
                            detector_cfg.gpio_md_adc_in != GPIO_NUM_NC,
                        ESP_ERR_INVALID_ARG,
                        TAG,
                        "invalid GPIO configuration");

    ESP_RETURN_ON_FALSE(detector_cfg.md_pulse_us > 0 && detector_cfg.md_blank_us > 0 &&
                            detector_cfg.md_deadtime_us > 0 && detector_cfg.start_delay_us > 0,
                        ESP_ERR_INVALID_ARG,
                        TAG,
                        "invalid metal detector timing");

    s_cfg = detector_cfg; // set global config

    ESP_RETURN_ON_ERROR( // setup GPIO and ADC
        gpio_adc_setup(),
        TAG,
        "gpio_adc_setup failed");

    BaseType_t ok = xTaskCreatePinnedToCore(&metal_task,
                                            "metal_detector",
                                            task_cfg.stack_depth,
                                            nullptr,
                                            task_cfg.priority,
                                            &s_task_handle,
                                            task_cfg.core_id);

    if (ok != pdPASS) {
        ESP_LOGE(TAG, "failed to instantiate task");
        return ESP_FAIL;
    }

    // setup timer
    esp_err_t err = timer_setup();

    if (err != ESP_OK) {
        vTaskDelete(s_task_handle); // timer setup failed... delete and throw
        s_task_handle = nullptr;

        ESP_LOGE(TAG, "timer_setup failed: %s", esp_err_to_name(err));
        return err;
    }

    if (out_handle != nullptr) {
        *out_handle = s_task_handle;
    }

    return ESP_OK;
}

void get_snapshot(MetalSnapshot &out)
{
    portENTER_CRITICAL(&s_mux);
    out = s_snapshot;
    portEXIT_CRITICAL(&s_mux);
}

void get_state(MetalState &out)
{
    portENTER_CRITICAL(&s_mux);
    out = s_snapshot.state;
    portEXIT_CRITICAL(&s_mux);
}
} // namespace metal_detector
