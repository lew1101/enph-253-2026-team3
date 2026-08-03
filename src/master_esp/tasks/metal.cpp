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

void sync_status_bits(EventBits_t mask, EventBits_t active_bits)
{
    xEventGroupClearBits(supervisor::g_robot_status_flags, mask);
    if (active_bits != 0) {
        xEventGroupSetBits(supervisor::g_robot_status_flags, active_bits);
    }
}

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

    if (s_task_handle != nullptr)
        vTaskNotifyGiveFromISR(s_task_handle, &higher_priority_task_woken);

    if (higher_priority_task_woken == pdTRUE) portYIELD_FROM_ISR();
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

    xEventGroupClearBits(supervisor::g_robot_status_flags,
                         robot_flags::STATUS_METAL_CALIBRATED_MASK |
                             robot_flags::STATUS_METAL_SEEN_MASK);

    MetalDetector md_1{METAL_1_CFG};
    MetalDetector md_2{METAL_2_CFG};

    if (timer_setup() != ESP_OK) {
        ESP_LOGE(TAG, "timer setup failed, deleting metal task");
        return vTaskDelete(nullptr);
    }

    if ((MD_1_ENABLE && md_1.init() != ESP_OK) || (MD_2_ENABLE && md_2.init() != ESP_OK)) {
        ESP_LOGE(TAG, "metal detectors failed to enable, deleting metal task");
        return vTaskDelete(nullptr);
    }

    while (true) {
        xEventGroupWaitBits(supervisor::g_robot_control_flags,
                            robot_flags::CONTROL_METAL_ENABLED,
                            pdFALSE,
                            pdTRUE,
                            portMAX_DELAY);

        // Remove any notification left from a previous enabled period.
        ulTaskNotifyTake(pdTRUE, 0);

        // reset the metal detector. this will cause them to recalibrate.
        if (MD_1_ENABLE) md_1.reset();
        if (MD_2_ENABLE) md_2.reset();

        xEventGroupClearBits(supervisor::g_robot_status_flags,
                             robot_flags::STATUS_METAL_CALIBRATED_MASK |
                                 robot_flags::STATUS_METAL_SEEN_MASK);

        arm_timer_us(MD_START_DELAY_US);

        MetalDetectorSnapshots snapshots;

        bool was_all_calibrated = false;
        bool was_any_metal_seen = false;
        bool sample_detector_1 = true; // if true, sample detector 1. else sample detector 2

        while (true) {
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

            if (MD_1_ENABLE && sample_detector_1) {
                md_1.pulse_and_sample();
            } else if (MD_2_ENABLE) {
                md_2.pulse_and_sample();
            }
            arm_timer_us(MD_STAGGER_US);

            if (MD_1_ENABLE && sample_detector_1) {
                md_1.update();
                md_1.get_snapshot(&snapshots.detector_1);
            } else if (MD_2_ENABLE) {
                md_2.update();
                md_2.get_snapshot(&snapshots.detector_2);
            }

            sample_detector_1 = !sample_detector_1;

            xQueueOverwrite(s_metal_snapshot_queue, &snapshots);

            EventBits_t calibrated_bits = 0;
            if (MD_1_ENABLE && md_1.is_calibration_complete()) {
                calibrated_bits |= robot_flags::STATUS_METAL_1_CALIBRATED;
            }
            if (MD_2_ENABLE && md_2.is_calibration_complete()) {
                calibrated_bits |= robot_flags::STATUS_METAL_2_CALIBRATED;
            }
            sync_status_bits(robot_flags::STATUS_METAL_CALIBRATED_MASK, calibrated_bits);

            const bool all_calibrated = robot_flags::has_all_flags(
                calibrated_bits, robot_flags::STATUS_METAL_CALIBRATED_MASK);
            if (all_calibrated && !was_all_calibrated) {
                ESP_ERROR_CHECK_WITHOUT_ABORT(
                    supervisor::notify_main(robot_flags::NOTIFY_METAL_CALIBRATED));
            }
            was_all_calibrated = all_calibrated;

            EventBits_t seen_bits = 0;
            if (snapshots.detector_1.state == MetalState::METAL_DETECTED) {
                seen_bits |= robot_flags::STATUS_METAL_1_SEEN;
            }
            if (snapshots.detector_2.state == MetalState::METAL_DETECTED) {
                seen_bits |= robot_flags::STATUS_METAL_2_SEEN;
            }

            sync_status_bits(robot_flags::STATUS_METAL_SEEN_MASK, seen_bits);

            const bool any_metal_seen = seen_bits != 0;
            if (any_metal_seen && !was_any_metal_seen) {
                ESP_ERROR_CHECK_WITHOUT_ABORT(
                    supervisor::notify_main(robot_flags::NOTIFY_METAL_FOUND));
            }
            was_any_metal_seen = any_metal_seen;

            const EventBits_t control_flags = xEventGroupGetBits(supervisor::g_robot_control_flags);
            if (!robot_flags::has_flag(control_flags, robot_flags::CONTROL_METAL_ENABLED)) {
                break;
            }
        }

        xEventGroupClearBits(supervisor::g_robot_status_flags,
                             robot_flags::STATUS_METAL_CALIBRATED_MASK |
                                 robot_flags::STATUS_METAL_SEEN_MASK);
    }
}
} // namespace

bool get_metal_detector_snapshots(MetalDetectorSnapshots *snapshots)
{
    configASSERT(s_metal_snapshot_queue != nullptr);
    configASSERT(snapshots != nullptr);
    return xQueuePeek(s_metal_snapshot_queue, snapshots, 0) == pdTRUE;
}

esp_err_t start_metal_detector_task(TaskHandle_t *out_handle)
{
    if (!MD_1_ENABLE && !MD_2_ENABLE) return ESP_FAIL;

    ESP_RETURN_ON_FALSE(MD_SAMPLE_PERIOD_US > 0 && MD_SAMPLE_PERIOD_US % 2 == 0 &&
                            MD_STAGGER_US > METAL_1_CFG.md_pulse_us + METAL_1_CFG.md_blank_us &&
                            MD_STAGGER_US > METAL_2_CFG.md_pulse_us + METAL_2_CFG.md_blank_us &&
                            MD_START_DELAY_US > 0,
                        ESP_ERR_INVALID_ARG,
                        TAG,
                        "invalid metal detector timing");

    if (s_task_handle != nullptr) {
        if (out_handle != nullptr) {
            *out_handle = s_task_handle;
        }

        return ESP_ERR_INVALID_STATE;
    }

    s_metal_snapshot_queue = xQueueCreate(1, sizeof(MetalDetectorSnapshots));
    configASSERT(s_metal_snapshot_queue != nullptr);

    // setup timer
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
