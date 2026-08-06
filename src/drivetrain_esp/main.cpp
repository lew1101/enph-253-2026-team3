#include <Arduino.h>
#include <cmath>
#include <cstring>

#include "freertos/idf_additions.h"

#include "tasks/drive.hpp"
#include "tasks/tape_sense.hpp"
#include "tasks/imu.hpp"
#include "tasks/uart.hpp"

static constexpr char TAG[]{"drivetrain_main"};

static TaskHandle_t uart_tx_handle;
static TaskHandle_t uart_rx_handle;
static TaskHandle_t drive_handle;
static TaskHandle_t imu_sensor_handle;
static TaskHandle_t tape_handle;

void setup()
{
    Serial.begin(SERIAL_BAUD);

    ESP_ERROR_CHECK_WITHOUT_ABORT(start_uart_tasks(&uart_tx_handle, &uart_rx_handle));
    ESP_ERROR_CHECK_WITHOUT_ABORT(start_imu_task(&imu_sensor_handle));
    ESP_ERROR_CHECK_WITHOUT_ABORT(start_drive_task(&drive_handle));
    ESP_ERROR_CHECK_WITHOUT_ABORT(start_tape_sense_task(&tape_handle));

    // vTaskDelete(nullptr);
}

// void loop()
// {
// }

static uint32_t seq = 0;

void send_pose_setpoint(const String &command, bool relative)
{
    static uint32_t seq = 0;

    float x_m = 0.0f;
    float y_m = 0.0f;
    float heading_deg = 0.0f;

    if (sscanf(command.c_str(), "%*c %f %f %f", &x_m, &y_m, &heading_deg) != 3 ||
        !std::isfinite(x_m) || !std::isfinite(y_m) || !std::isfinite(heading_deg)) {
        ESP_LOGW(TAG, "use: p <x_m> <y_m> <heading_deg> or r <dx_m> <dy_m> <dheading_deg>");
        return;
    }

    robot_DriveCommand pose_cmd = robot_DriveCommand_init_zero;
    pose_cmd.which_command = robot_DriveCommand_pose_tag;
    pose_cmd.command.pose.x_m = x_m;
    pose_cmd.command.pose.y_m = y_m;
    pose_cmd.command.pose.theta_rad = radians(heading_deg);
    pose_cmd.command.pose.relative = relative;
    pose_cmd.sequence = ++seq;

    if (send_drive_cmd(pose_cmd) != ESP_OK) {
        ESP_LOGE(TAG, "failed to send pose setpoint");
        return;
    }

    ESP_LOGI(TAG,
             "%s pose setpoint: x=%.4f m y=%.4f m heading=%.4f deg",
             relative ? "relative" : "absolute",
             x_m,
             y_m,
             heading_deg);
}

void send_velocity(float vx_percent, float vy_percent, float omega_percent = 0.0f)
{
    robot_DriveCommand velocity_cmd = robot_DriveCommand_init_zero;
    velocity_cmd.which_command = robot_DriveCommand_velocity_tag;
    velocity_cmd.command.velocity.vx_percent = vx_percent;
    velocity_cmd.command.velocity.vy_percent = vy_percent;
    velocity_cmd.command.velocity.omega_percent = omega_percent;
    velocity_cmd.sequence = ++seq;

    if (send_drive_cmd(velocity_cmd) == ESP_OK) {
        ESP_LOGI("drive_pose",
                 "velocity: x=%.2f%% y=%.2f%% turn=%.2f%%",
                 vx_percent,
                 vy_percent,
                 omega_percent);
    } else {
        ESP_LOGE("drive_pose", "failed to send velocity command");
    }
}

void send_stop()
{
    robot_DriveCommand stop_cmd = robot_DriveCommand_init_zero;
    stop_cmd.which_command = robot_DriveCommand_stop_tag;
    stop_cmd.command.stop.brake = true;
    stop_cmd.sequence = ++seq;

    if (send_drive_cmd(stop_cmd) == ESP_OK) {
        ESP_LOGI("drive_pose", "stopped");
    } else {
        ESP_LOGE("drive_pose", "failed to send stop command");
    }
}

bool parse_speed(const String &command, const char *prefix, float *out_speed)
{
    if (!command.startsWith(prefix) || out_speed == nullptr) return false;

    const String value = command.substring(strlen(prefix));
    char trailing = '\0';
    float speed = 0.0f;
    if (sscanf(value.c_str(), "%f %c", &speed, &trailing) != 1 || !std::isfinite(speed) ||
        speed < 0.0f || speed > 100.0f) {
        ESP_LOGW(TAG, "speed must be between 0 and 100 percent");
        return false;
    }

    *out_speed = speed;
    return true;
}

void loop()
{
    if (Serial.available() <= 0) {
        delay(10);
        return;
    }

    String command = Serial.readStringUntil('\n');
    command.trim();

    float speed = 0.0f;
    if (command.startsWith("p ")) {
        send_pose_setpoint(command, false);
    } else if (command.startsWith("r ")) {
        send_pose_setpoint(command, true);
    } else if (parse_speed(command, "forward ", &speed)) {
        send_velocity(0.0f, speed);
    } else if (parse_speed(command, "backward ", &speed)) {
        send_velocity(0.0f, -speed);
    } else if (parse_speed(command, "strafe left ", &speed)) {
        send_velocity(-speed, 0.0f);
    } else if (parse_speed(command, "strafe right ", &speed)) {
        send_velocity(speed, 0.0f);
    } else if (parse_speed(command, "turn left ", &speed)) {
        send_velocity(0.0f, 0.0f, speed);
    } else if (parse_speed(command, "turn right ", &speed)) {
        send_velocity(0.0f, 0.0f, -speed);
    } else if (command == "stop" || command == "s" || command == "x") {
        send_stop();
    } else if (command == "?") {
        ESP_LOGI(TAG, "g <x|y|h> <kp> <ki> <kd>");
        ESP_LOGI(TAG, "p <x_m> <y_m> <heading_deg>  (absolute pose)");
        ESP_LOGI(TAG, "r <dx_m> <dy_m> <dheading_deg> (relative pose)");
        ESP_LOGI(TAG, "forward <percent>");
        ESP_LOGI(TAG, "backward <percent>");
        ESP_LOGI(TAG, "strafe left <percent>");
        ESP_LOGI(TAG, "strafe right <percent>");
        ESP_LOGI(TAG, "turn left <percent>");
        ESP_LOGI(TAG, "turn right <percent>");
        ESP_LOGI(TAG, "stop                           (aliases: s, x)");
    } else if (!command.isEmpty()) {
        ESP_LOGW(TAG, "unknown command; enter ? for help");
    }
}
