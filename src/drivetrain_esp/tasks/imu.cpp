#include <Arduino.h>
#include <Wire.h>

#include <SparkFun_BNO08x_Arduino_Library.h>

#include "tasks/imu.hpp"
#include "freertos/idf_additions.h"

using namespace ImuTaskConfig;

static constexpr char TAG[] = "imu_task";

namespace {
TaskHandle_t s_task_handle;
QueueHandle_t s_snapshot_queue;
uint32_t s_reset_count = 0;

BNO08x s_imu;

inline bool _enable_rot_vec() { return s_imu.enableGameRotationVector(REPORT_PERIOD_MS); }

void ARDUINO_ISR_ATTR _imu_int_isr(void *)
{
    if (s_task_handle == nullptr) return;

    BaseType_t higher_priority_task_woken = pdFALSE;
    vTaskNotifyGiveFromISR(s_task_handle, &higher_priority_task_woken);
    portYIELD_FROM_ISR(higher_priority_task_woken);
}

void _imu_task(void *arg)
{
    (void)arg;
    Wire.begin(SDA_PIN, SCL_PIN);
    Wire.setClock(IMU_I2C_FREQ_HZ);

    pinMode(INT_PIN, INPUT_PULLUP);
    attachInterruptArg(INT_PIN, _imu_int_isr, nullptr, FALLING);

    // This task owns INT, so the library must not poll it internally.
    if (!s_imu.begin(IMU_I2C_ADDRESS, Wire, -1, RST_PIN)) {
        ESP_LOGE(TAG, "BNO086 not detected");
        detachInterrupt(INT_PIN);
        s_task_handle = nullptr;
        vTaskDelete(nullptr);
        return;
    }

    if (!_enable_rot_vec()) {
        ESP_LOGE(TAG, "failed to enable game rotation vector");
        detachInterrupt(INT_PIN);
        s_task_handle = nullptr;
        vTaskDelete(nullptr);
        return;
    }

    s_imu.tareNow();

    ESP_LOGI(TAG, "BNO086 initialized");

    while (true) {
        const uint32_t notif = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(20));

        // Recover if an edge is missed while the active-low INT line remains asserted.
        if (notif == 0 && digitalRead(INT_PIN) != LOW) continue;

        while (s_imu.getSensorEvent()) {
            if (s_imu.wasReset()) {
                ESP_LOGW(TAG, "IMU reset; re-enabling reports");
                ++s_reset_count;
                if (!_enable_rot_vec()) {
                    ESP_LOGE(TAG, "failed to re-enable game rotation vector");
                }
            }

            if (s_imu.getSensorEventID() == SENSOR_REPORTID_GAME_ROTATION_VECTOR) {
                float yaw = s_imu.getYaw();
                float pitch = s_imu.getPitch();
                float roll = s_imu.getRoll();

                // log_d(
                //     "yaw=%.2f deg, pitch=%.2f deg, roll=%.2f deg", degrees(yaw), degrees(pitch),
                //     degrees(roll));

                ImuSnapshot snapshot{
                    .yaw = yaw,
                    .pitch = pitch,
                    .roll = roll,
                    .tick = xTaskGetTickCount(),
                    .reset_count = s_reset_count,
                    .valid = true,
                };

                xQueueOverwrite(s_snapshot_queue, &snapshot);
            }
        }
    }
}
} // namespace

esp_err_t start_imu_task(TaskHandle_t *out_handle)
{
    if (s_task_handle != nullptr) {
        if (out_handle != nullptr) {
            *out_handle = s_task_handle;
        }

        return ESP_ERR_INVALID_STATE;
    }

    s_snapshot_queue = xQueueCreate(1, sizeof(ImuSnapshot));
    configASSERT(s_snapshot_queue != nullptr);

    auto ok = xTaskCreatePinnedToCore(_imu_task,
                                      "imu_task",
                                      TASK_STACK_DEPTH,
                                      nullptr,
                                      TASK_PRIORITY,
                                      &s_task_handle,
                                      TASK_CORE_ID);

    if (ok != pdPASS) {
        ESP_LOGE(TAG, "failed to instantiate drive task");

        s_task_handle = nullptr;
        return ESP_FAIL;
    }

    if (out_handle != nullptr) {
        *out_handle = s_task_handle;
    }

    return ESP_OK;
}

bool get_imu_snapshot(ImuSnapshot *out, TickType_t timeout)
{
    if (out == nullptr || s_snapshot_queue == nullptr) return false;
    return xQueuePeek(s_snapshot_queue, out, timeout) == pdTRUE;
}
