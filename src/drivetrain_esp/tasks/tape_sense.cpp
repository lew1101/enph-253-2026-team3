#include "freertos/idf_additions.h"
#include "tasks/tape_sense.hpp"

#include "esp_err.h"

static constexpr const char TAG[] = "tape_task";

namespace {
TaskHandle_t s_task_handle = nullptr;
QueueHandle_t s_snapshot_queue;

TapeSnapshot s_snapshot;
TapeSenseTaskConfig s_task_config;

float fl_adc_val = 0.0f;
float fm_adc_val = 0.0f;
float fr_adc_val = 0.0f;

float bl_adc_val = 0.0f;
float bm_adc_val = 0.0f;
float br_adc_val = 0.0f;

float l1_adc_val = 0.0f;
float l2_adc_val = 0.0f;

float ALPHA = 1.0f;
float outer_error = 5.0f;

bool _apply_hyteresis(bool sees_tape, float adc_val)
{
    if (sees_tape && adc_val < s_task_config.low_threshold)
        return false;
    else if (!sees_tape && adc_val > s_task_config.high_threshold)
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
            return outer_error; // Last known position was to the right
        } else if (prev_error < 0.0f) {
            return -outer_error; // Last known position was to the left
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
    gpio_set_direction(s_task_config.fl_tape_pin, GPIO_MODE_INPUT);
    gpio_set_direction(s_task_config.fm_tape_pin, GPIO_MODE_INPUT);
    gpio_set_direction(s_task_config.fr_tape_pin, GPIO_MODE_INPUT);

    gpio_set_direction(s_task_config.bl_tape_pin, GPIO_MODE_INPUT);
    gpio_set_direction(s_task_config.bm_tape_pin, GPIO_MODE_INPUT);
    gpio_set_direction(s_task_config.br_tape_pin, GPIO_MODE_INPUT);

    gpio_set_direction(s_task_config.l1_tape_pin, GPIO_MODE_INPUT);
    gpio_set_direction(s_task_config.l2_tape_pin, GPIO_MODE_INPUT);

    while (true) {
        // xEventGroupWaitBits(
        //     g_robot_flags, RobotFlag::ROBOT_FLAG_TAPE_ACTIVE, pdFALSE, pdTRUE, portMAX_DELAY);

        fl_adc_val = analogRead(s_task_config.fl_tape_pin);
        fm_adc_val = analogRead(s_task_config.fm_tape_pin);
        fr_adc_val = analogRead(s_task_config.fr_tape_pin);

        bl_adc_val = analogRead(s_task_config.bl_tape_pin);
        bm_adc_val = analogRead(s_task_config.bm_tape_pin);
        br_adc_val = analogRead(s_task_config.br_tape_pin);

        l1_adc_val = analogRead(s_task_config.l1_tape_pin);
        l2_adc_val = analogRead(s_task_config.l2_tape_pin);

        s_snapshot.tape_fl = _apply_hyteresis(s_snapshot.tape_fl, fl_adc_val);
        s_snapshot.tape_fm = _apply_hyteresis(s_snapshot.tape_fm, fm_adc_val);
        s_snapshot.tape_fr = _apply_hyteresis(s_snapshot.tape_fr, fr_adc_val);

        s_snapshot.tape_bl = _apply_hyteresis(s_snapshot.tape_bl, bl_adc_val);
        s_snapshot.tape_bm = _apply_hyteresis(s_snapshot.tape_bm, bm_adc_val);
        s_snapshot.tape_br = _apply_hyteresis(s_snapshot.tape_br, br_adc_val);

        s_snapshot.tape_l1 = _apply_hyteresis(s_snapshot.tape_l1, l1_adc_val);
        s_snapshot.tape_l2 = _apply_hyteresis(s_snapshot.tape_l2, l2_adc_val);

        float new_front_err = _get_tape_error(s_snapshot.tape_fl, s_snapshot.tape_fr, s_snapshot.front_err);
        s_snapshot.front_err = ALPHA * new_front_err + (1 - ALPHA) * s_snapshot.front_err;

        // using two tape sensors logic
        float new_back_err = _get_tape_error(s_snapshot.tape_bl, s_snapshot.tape_br, s_snapshot.back_err);
        s_snapshot.back_err = ALPHA * new_back_err + (1 - ALPHA) * s_snapshot.back_err;

        float new_left_err = _get_tape_error(s_snapshot.tape_l1, s_snapshot.tape_l2, s_snapshot.left_err);
        s_snapshot.left_err = ALPHA * new_left_err + (1 - ALPHA) * s_snapshot.left_err;

        s_snapshot.tick = xTaskGetTickCount();
        s_snapshot.valid = true;

        xQueueOverwrite(s_snapshot_queue, &s_snapshot);

        vTaskDelay(pdMS_TO_TICKS(static_cast<uint32_t>(s_task_config.period_ms)));
    }
}

} // namespace

void change_outer_error(float new_outer_error)
{
    outer_error = new_outer_error;
}

esp_err_t start_tape_sense_task(const TapeSenseTaskConfig &task_config, TaskHandle_t *out_handle)
{
    if (s_task_handle != nullptr) {
        if (out_handle != nullptr) {
            *out_handle = s_task_handle;
        }

        return ESP_ERR_INVALID_STATE;
    }

    s_task_config = task_config;

    s_snapshot_queue = xQueueCreate(1, sizeof(TapeSnapshot));
    configASSERT(s_snapshot_queue != nullptr);

    auto ok = xTaskCreatePinnedToCore(_tape_task,
                                      "tape_task",
                                      s_task_config.stack_depth,
                                      nullptr,
                                      s_task_config.priority,
                                      &s_task_handle,
                                      s_task_config.core_id);

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
