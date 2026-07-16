#include <Arduino.h>
#include <Wire.h>

#include <SparkFun_BNO08x_Arduino_Library.h>

#include "tasks/imu.hpp"
#include "freertos/idf_additions.h"

static constexpr char TAG[] = "imu_task";

namespace {
TaskHandle_t s_task_handle;
QueueHandle_t s_snapshot_queue;

ImuTaskConfig s_task_cfg;
BNO08x s_imu;

inline bool _enable_rot_vec() { return s_imu.enableRotationVector(s_task_cfg.report_period_ms); }

void _imu_task(void *arg)
{
    (void)arg;
    Wire.begin(s_task_cfg.sda_pin, s_task_cfg.scl_pin);
    Wire.setClock(IMU_I2C_FREQ);

    if (!s_imu.begin(IMU_I2C_ADDRESS, Wire, s_task_cfg.int_pin, s_task_cfg.rst_pin)) {
        ESP_LOGE(TAG, "BNO086 not detected");
        vTaskDelete(nullptr);
    }

    if (!_enable_rot_vec()) {
        ESP_LOGE(TAG, "failed to enable rotation vector");
        vTaskDelete(nullptr);
    }

    ESP_LOGI(TAG, "BNO086 initialized");

    while (true) {
        if (s_imu.wasReset()) {
            ESP_LOGW("imu", "IMU reset; re-enabling reports");
            _enable_rot_vec();
        }

        if (s_imu.getSensorEvent() && s_imu.getSensorEventID() == SENSOR_REPORTID_ROTATION_VECTOR) {
            float yaw = s_imu.getYaw();
            float pitch = s_imu.getPitch();
            float roll = s_imu.getRoll();

            if (yaw < 0.0f) yaw += TWO_PI;
            if (pitch < 0.0f) pitch += TWO_PI;
            if (roll < 0.0f) roll += TWO_PI;

            ESP_LOGV(TAG, "yaw=%.2f, pitch=%.2f, roll=%.2f", yaw, pitch, roll);

            ImuSnapshot snapshot{
                .yaw = yaw,
                .pitch = pitch,
                .roll = roll,
                .tick = xTaskGetTickCount(),
                .valid = true,
            };

            xQueueOverwrite(s_snapshot_queue, &snapshot);
        }

        vTaskDelay(pdMS_TO_TICKS(1));
    }
}
} // namespace

esp_err_t start_tape_sense_task(const ImuTaskConfig &task_cfg, TaskHandle_t *out_handle)
{
    if (s_task_handle != nullptr) {
        if (out_handle != nullptr) {
            *out_handle = s_task_handle;
        }

        return ESP_ERR_INVALID_STATE;
    }

    s_task_cfg = task_cfg;

    s_snapshot_queue = xQueueCreate(1, sizeof(ImuSnapshot));
    configASSERT(s_snapshot_queue != nullptr);

    auto ok = xTaskCreatePinnedToCore(_imu_task,
                                      "imu_task",
                                      s_task_cfg.stack_depth,
                                      nullptr,
                                      s_task_cfg.priority,
                                      &s_task_handle,
                                      s_task_cfg.core_id);

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
    return xQueuePeek(s_snapshot_queue, out, timeout) == pdTRUE;
}
