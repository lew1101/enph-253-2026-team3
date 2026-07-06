#ifdef DEBUG
#define LOG_LEVEL_LOCAL ESP_LOG_VERBOSE
#endif

#include "tasks/metal.hpp"

#include "esp32-hal-timer.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_check.h"

static constexpr char TAG[] = "metal_task";

using namespace metal_detector;

namespace {
constexpr uint32_t TIMER_FREQ = 1000000;

hw_timer_t *s_md_timer = nullptr;
TaskHandle_t s_task_handle = nullptr;

MetalDetector *s_md = nullptr;
MetalTaskConfig s_task_cfg;

inline void IRAM_ATTR arm_timer_us(uint32_t delay_us)
{
    timerWrite(s_md_timer, 0); // reset timer
    timerAlarm(s_md_timer, delay_us, false, 0);
}

void IRAM_ATTR on_metal_timer()
{
    BaseType_t higher_priority_task_woken = pdFALSE;

    if (s_task_handle != nullptr) {
        vTaskNotifyGiveFromISR(s_task_handle, &higher_priority_task_woken);
    }

    if (higher_priority_task_woken == pdTRUE) {
        portYIELD_FROM_ISR();
    }
}

esp_err_t timer_setup()
{
    // setup timer
    s_md_timer = timerBegin(TIMER_FREQ);

    if (s_md_timer == nullptr) {
        return ESP_FAIL;
    }

    timerAttachInterrupt(s_md_timer, &on_metal_timer); // attach interrupt handler

    return ESP_OK;
}

void metal_task(void *arg)
{
    (void)arg;

    arm_timer_us(s_task_cfg.start_delay_us);

    while (true) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        s_md->pulse_and_sample();
        arm_timer_us(s_task_cfg.md_deadtime_us);
        s_md->update(); // update during the deadtime to give more consistent timing.
    }
}
} // namespace

esp_err_t start_metal_detector_task(const MetalDetector::Config &cfg,
                                    const MetalTaskConfig &task_cfg,
                                    TaskHandle_t *out_handle)
{
    s_task_cfg = task_cfg;

    ESP_RETURN_ON_FALSE(s_task_cfg.md_deadtime_us > 0 && s_task_cfg.start_delay_us > 0,
                        ESP_ERR_INVALID_ARG,
                        TAG,
                        "invalid metal detector timing");

    if (s_task_handle != nullptr) {
        if (out_handle != nullptr) {
            *out_handle = s_task_handle;
        }

        return ESP_ERR_INVALID_STATE;
    }

    // initilize metal detector and pass it to static pointer
    static MetalDetector md{cfg};
    s_md = &md;

    // set up metal_detector
    ESP_RETURN_ON_ERROR(s_md->init(), TAG, "metal detector setup failed");
    // setup timer
    ESP_RETURN_ON_ERROR(timer_setup(), TAG, "timer setup failed");

    BaseType_t ok = xTaskCreatePinnedToCore(&metal_task,
                                            "metal_task",
                                            s_task_cfg.stack_depth,
                                            nullptr,
                                            s_task_cfg.priority,
                                            &s_task_handle,
                                            s_task_cfg.core_id);

    if (ok != pdPASS) {
        ESP_LOGE(TAG, "failed to instantiate metal task");

        if (s_md_timer != nullptr) {
            timerEnd(s_md_timer);
            s_md_timer = nullptr;
        }

        s_task_handle = nullptr;
        return ESP_FAIL;
    }

    if (out_handle != nullptr) {
        *out_handle = s_task_handle;
    }

    return ESP_OK;
}
