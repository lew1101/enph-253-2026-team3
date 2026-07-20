#include "freertos/idf_additions.h"
#include "portmacro.h"
#include "esp32-hal-timer.h"

#include "esp_log.h"
#include "esp_err.h"
#include "esp_check.h"

#include "tasks/metal.hpp"
#include "sensors/metal_detector.hpp"
#include "shared/robot_flags.hpp"
#include "supervisor.hpp"

static constexpr char TAG[] = "metal_task";

using namespace MetalTaskConfig;

using namespace metal_detector;

namespace {
TaskHandle_t s_task_handle = nullptr;

hw_timer_t *s_md_timer = nullptr;

QueueHandle_t s_metal_snapshot_queue = nullptr;
MetalDetector s_md{METAL_CFG};

inline void IRAM_ATTR arm_timer_us(uint32_t delay_us)
{
    configASSERT(s_md_timer != nullptr);
    configASSERT(delay_us > 0);

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
    ESP_RETURN_ON_FALSE(s_md_timer != nullptr, ESP_ERR_NO_MEM, TAG, "failed to create metal timer");

    timerAttachInterrupt(s_md_timer, &on_metal_timer); // attach interrupt handler
    return ESP_OK;
}

void timer_cleanup()
{
    if (s_md_timer != nullptr) {
        timerEnd(s_md_timer);
        s_md_timer = nullptr;
    }
}

void metal_task(void *arg)
{
    (void)arg;

    xEventGroupClearBits(supervisor::g_robot_status_flags, robot_flags::STATUS_METAL_CALIBRATED);

    MetalDetector::Snapshot snapshot;
    bool finish_calibration = false;

    while (true) {
        xEventGroupWaitBits(supervisor::g_robot_control_flags,
                            robot_flags::CONTROL_METAL_ENABLED,
                            pdFALSE,
                            pdTRUE,
                            portMAX_DELAY);

        // Remove any notification left from a previous enabled period.
        ulTaskNotifyTake(pdTRUE, 0);

        arm_timer_us(MD_START_DELAY_US);

        while (true) {
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

            const EventBits_t status_flags = xEventGroupGetBits(supervisor::g_robot_status_flags);

            s_md.pulse_and_sample();
            arm_timer_us(MD_DEADTIME_US);
            s_md.update(); // update during the deadtime to give more consistent timing.

            s_md.get_snapshot(&snapshot);
            xQueueOverwrite(s_metal_snapshot_queue, &snapshot);

            if (finish_calibration == false && s_md.is_calibration_complete()) {
                finish_calibration = true;
                xEventGroupSetBits(supervisor::g_robot_status_flags,
                                   robot_flags::STATUS_METAL_CALIBRATED);
                ESP_ERROR_CHECK_WITHOUT_ABORT(
                    supervisor::notify_main(robot_flags::NOTIFY_METAL_CALIBRATED));
            }

            bool metal_seen = snapshot.state == MetalState::METAL_DETECTED;
            if (metal_seen) {
                // metal seen!
                xEventGroupSetBits(supervisor::g_robot_status_flags,
                                   robot_flags::STATUS_METAL_SEEN);
                ESP_ERROR_CHECK_WITHOUT_ABORT(
                    supervisor::notify_main(robot_flags::NOTIFY_METAL_FOUND));
            } else {
                // no metal :(
                xEventGroupClearBits(supervisor::g_robot_status_flags,
                                     robot_flags::STATUS_METAL_SEEN);
            }
        }
    }
}
} // namespace

bool get_metal_detector_snapshot(MetalDetector::Snapshot &snapshot)
{
    configASSERT(s_metal_snapshot_queue != nullptr);
    return xQueuePeek(s_metal_snapshot_queue, &snapshot, 0) == pdTRUE;
}

esp_err_t start_metal_detector_task(TaskHandle_t *out_handle)
{
    ESP_RETURN_ON_FALSE(MD_DEADTIME_US > 0 && MD_START_DELAY_US > 0,
                        ESP_ERR_INVALID_ARG,
                        TAG,
                        "invalid metal detector timing");

    if (s_task_handle != nullptr) {
        if (out_handle != nullptr) {
            *out_handle = s_task_handle;
        }

        return ESP_ERR_INVALID_STATE;
    }

    s_metal_snapshot_queue = xQueueCreate(1, sizeof(MetalDetector::Snapshot));
    configASSERT(s_metal_snapshot_queue != nullptr);

    // set up metal_detector
    ESP_RETURN_ON_ERROR(s_md.init(), TAG, "metal detector setup failed");
    // setup timer
    ESP_RETURN_ON_ERROR(timer_setup(), TAG, "timer setup failed");

    BaseType_t ok = xTaskCreatePinnedToCore(&metal_task,
                                            "metal_task",
                                            TASK_STACK_DEPTH,
                                            nullptr,
                                            TASK_PRIORITY,
                                            &s_task_handle,
                                            TASK_CORE_ID);

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
