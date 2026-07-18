#include "freertos/idf_additions.h"
#include "tasks/tape_sense.hpp"

#include "esp_err.h"
#include "portmacro.h"

static constexpr const char TAG[] = "tape_task";

using namespace TapeSenseTaskConfig;

namespace {
TaskHandle_t s_task_handle = nullptr;
QueueHandle_t s_snapshot_queue;

TapeSnapshot s_snapshot;

bool _apply_hyteresis(bool sees_tape, float adc_val)
{
    if (sees_tape && adc_val < TAPE_LOW_THRESHOLD)
        return false;
    else if (!sees_tape && adc_val > TAPE_HIGH_THRESHOLD)
        return true;
    return sees_tape;
}

float _get_tape_error(bool left, bool right, float prev_error)
{
    if (left && !right) {
        return 1.0f; // Left sensor sees tape, robot is too far right
    } else if (!left && right) {
        return -1.0f; // Right sensor sees tape, robot is too far left
    } else if (!left && !right) {
        // Both sensors do not see tape, use previous error to determine direction
        if (prev_error > 0.0f) {
            return 5.0f; // Last known position was to the right
        } else if (prev_error < 0.0f) {
            return -5.0f; // Last known position was to the left
        }
    }

    return 0.0f;
}

float _get_tape_error(bool left, bool middle, bool right, float prev_error)
{
    // TODO
    return 0.0;
}

void _tape_task(void *arg)
{
    (void)arg;

    // Define the GPIO pins for the tape sensors
    gpio_set_direction(FL_TAPE_PIN, GPIO_MODE_INPUT);
    gpio_set_direction(FM_TAPE_PIN, GPIO_MODE_INPUT);
    gpio_set_direction(FR_TAPE_PIN, GPIO_MODE_INPUT);

    gpio_set_direction(BL_TAPE_PIN, GPIO_MODE_INPUT);
    gpio_set_direction(BM_TAPE_PIN, GPIO_MODE_INPUT);
    gpio_set_direction(BR_TAPE_PIN, GPIO_MODE_INPUT);

    gpio_set_direction(L1_TAPE_PIN, GPIO_MODE_INPUT);
    gpio_set_direction(L2_TAPE_PIN, GPIO_MODE_INPUT);

    float fl_adc_val = 0.0f;
    float fm_adc_val = 0.0f;
    float fr_adc_val = 0.0f;

    float bl_adc_val = 0.0f;
    float bm_adc_val = 0.0f;
    float br_adc_val = 0.0f;

    float l1_adc_val = 0.0f;
    float l2_adc_val = 0.0f;

    TickType_t last_wake_time = xTaskGetTickCount();

    while (true) {
        // xEventGroupWaitBits(
        //     g_robot_flags, RobotFlag::ROBOT_FLAG_TAPE_ACTIVE, pdFALSE, pdTRUE, portMAX_DELAY);

        fl_adc_val = analogRead(FL_TAPE_PIN);
        fm_adc_val = analogRead(FM_TAPE_PIN);
        fr_adc_val = analogRead(FR_TAPE_PIN);

        bl_adc_val = analogRead(BL_TAPE_PIN);
        bm_adc_val = analogRead(BM_TAPE_PIN);
        br_adc_val = analogRead(BR_TAPE_PIN);

        l1_adc_val = analogRead(L1_TAPE_PIN);
        l2_adc_val = analogRead(L2_TAPE_PIN);

        s_snapshot.tape_fl = _apply_hyteresis(s_snapshot.tape_fl, fl_adc_val);
        s_snapshot.tape_fm = _apply_hyteresis(s_snapshot.tape_fm, fm_adc_val);
        s_snapshot.tape_fr = _apply_hyteresis(s_snapshot.tape_fr, fr_adc_val);

        s_snapshot.tape_bl = _apply_hyteresis(s_snapshot.tape_bl, bl_adc_val);
        s_snapshot.tape_bm = _apply_hyteresis(s_snapshot.tape_bm, bm_adc_val);
        s_snapshot.tape_br = _apply_hyteresis(s_snapshot.tape_br, br_adc_val);

        s_snapshot.tape_l1 = _apply_hyteresis(s_snapshot.tape_l1, l1_adc_val);
        s_snapshot.tape_l2 = _apply_hyteresis(s_snapshot.tape_l2, l2_adc_val);

        s_snapshot.front_err = _get_tape_error(
            s_snapshot.tape_fl, s_snapshot.tape_fm, s_snapshot.tape_fr, s_snapshot.front_err);

        s_snapshot.back_err = _get_tape_error(
            s_snapshot.tape_bl, s_snapshot.tape_bm, s_snapshot.tape_br, s_snapshot.back_err);

        s_snapshot.left_err =
            _get_tape_error(s_snapshot.tape_l1, s_snapshot.tape_l2, s_snapshot.left_err);

        s_snapshot.tick = xTaskGetTickCount();
        s_snapshot.valid = true;

        xQueueOverwrite(s_snapshot_queue, &s_snapshot);

        // s_tape_error.store(error);
        // ESP_LOGI("TapeSense", "FL: %d, FR: %d, Error: %.2f", FL_sees_tape, FR_sees_tape, error);

        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(TASK_PERIOD_MS));
    }
}

} // namespace

esp_err_t start_tape_sense_task(TaskHandle_t *out_handle)
{
    if (s_task_handle != nullptr) {
        if (out_handle != nullptr) {
            *out_handle = s_task_handle;
        }

        return ESP_ERR_INVALID_STATE;
    }

    s_snapshot_queue = xQueueCreate(1, sizeof(TapeSnapshot));
    configASSERT(s_snapshot_queue != nullptr);

    auto ok = xTaskCreatePinnedToCore(_tape_task,
                                      "tape_task",
                                      TASK_STACK_DEPTH,
                                      nullptr,
                                      TASK_PRIORITY,
                                      &s_task_handle,
                                      TASK_CORE_ID);

    if (ok != pdPASS) {
        ESP_LOGE(TAG, "failed to instantiate tape task");

        s_task_handle = nullptr;
        return ESP_FAIL;
    }

    if (out_handle != nullptr) {
        *out_handle = s_task_handle;
    }
    return ESP_OK;
}

bool get_tape_snapshot(TapeSnapshot *out, TickType_t timeout)
{
    return xQueuePeek(s_snapshot_queue, out, timeout) == pdTRUE;
}
