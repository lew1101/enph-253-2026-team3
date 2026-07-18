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

BNO08x s_imu;

inline bool _enable_rot_vec() { return s_imu.enableRotationVector(REPORT_PERIOD_MS); }

void _imu_task(void *arg)
{
    (void)arg;
    Wire.begin(SDA_PIN, SCL_PIN);
    Wire.setClock(IMU_I2C_FREQ_HZ);

    if (!s_imu.begin(IMU_I2C_ADDRESS, Wire, INT_PIN, RST_PIN)) {
        ESP_LOGE(TAG, "BNO086 not detected");
        s_task_handle = nullptr;
        vTaskDelete(nullptr);
    }

    if (!_enable_rot_vec()) {
        ESP_LOGE(TAG, "failed to enable rotation vector");
        s_task_handle = nullptr;
        vTaskDelete(nullptr);
    }

    ESP_LOGI(TAG, "BNO086 initialized");

    while (true) {
        if (s_imu.wasReset()) {
            ESP_LOGW(TAG, "IMU reset; re-enabling reports");
            _enable_rot_vec();
        }

        if (s_imu.getSensorEvent() && s_imu.getSensorEventID() == SENSOR_REPORTID_ROTATION_VECTOR) {
            float yaw = s_imu.getYaw();
            float pitch = s_imu.getPitch();
            float roll = s_imu.getRoll();

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
    return xQueuePeek(s_snapshot_queue, out, timeout) == pdTRUE;
}
