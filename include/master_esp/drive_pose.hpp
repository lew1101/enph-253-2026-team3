#pragma once

#include "Arduino.h"
#include "esp_log.h"
#include "drive.pb.h"

#include "supervisor.hpp"
#include "tasks/uart.hpp"
#include "control/pose_estimator.hpp"

using control::Pose;

constexpr float DEFAULT_PATH_SPEED_MPS = 0.5f;

inline bool drive_fault_active()
{
    if (supervisor::g_robot_status_flags == nullptr) return true;
    const EventBits_t status = xEventGroupGetBits(supervisor::g_robot_status_flags);
    return robot_flags::has_any_flag(
        status, robot_flags::STATUS_DRIVE_FAULT_ACTIVE | robot_flags::STATUS_ESTOP_ACTIVE);
}

constexpr inline robot_DriveCommand make_pose_drive_command(
    const Pose &pose,
    bool is_relative = false,
    float desired_speed_mps = DEFAULT_PATH_SPEED_MPS)
{
    robot_DriveCommand out = robot_DriveCommand_init_zero;

    out.which_command = robot_DriveCommand_pose_tag;
    out.command.pose.relative = is_relative;
    out.command.pose.x_m = pose.x_m;
    out.command.pose.y_m = pose.y_m;
    out.command.pose.theta_rad = pose.heading_rad;
    out.command.pose.desired_speed_mps = desired_speed_mps;

    return out;
}

inline void send_velocity(float vx_percent, float vy_percent, float omega_percent = 0.0f)
{
    robot_DriveCommand velocity_cmd = robot_DriveCommand_init_zero;
    velocity_cmd.which_command = robot_DriveCommand_velocity_tag;
    velocity_cmd.command.velocity.vx_percent = vx_percent;
    velocity_cmd.command.velocity.vy_percent = vy_percent;
    velocity_cmd.command.velocity.omega_percent = omega_percent;

    if (send_drive_command(velocity_cmd) == ESP_OK) {
        ESP_LOGI("drive_pose",
                 "velocity: x=%.2f%% y=%.2f%% turn=%.2f%%",
                 vx_percent,
                 vy_percent,
                 omega_percent);
    } else {
        ESP_LOGE("drive_pose", "failed to send velocity command");
    }
}

inline void send_stop()
{
    robot_DriveCommand stop_cmd = robot_DriveCommand_init_zero;
    stop_cmd.which_command = robot_DriveCommand_stop_tag;
    stop_cmd.command.stop.brake = true;

    if (send_drive_command(stop_cmd) == ESP_OK) {
        ESP_LOGI("drive_pose", "stopped");
    } else {
        ESP_LOGE("drive_pose", "failed to send stop command");
    }
}

inline esp_err_t send_pose(const Pose &waypoint,
                           bool is_relative = false,
                           uint32_t *out_sequence = nullptr,
                           float desired_speed_mps = DEFAULT_PATH_SPEED_MPS)
{
    robot_DriveCommand cmd = make_pose_drive_command(waypoint, is_relative, desired_speed_mps);
    return send_drive_command(cmd, portMAX_DELAY, out_sequence);
}

inline esp_err_t wait_for_drive_sequence(uint32_t sequence, TickType_t timeout)
{
    TimeOut_t timeout_state;
    TickType_t remaining = timeout;
    vTaskSetTimeOutState(&timeout_state);

    while (!drive_command_completed(sequence)) {
        if (drive_fault_active()) return ESP_FAIL;

        if (!supervisor::wait_for_notification(robot_flags::NOTIFY_DRIVE_TARGET_REACHED,
                                               remaining)) {
            if (drive_fault_active()) return ESP_FAIL;
            return drive_command_completed(sequence) ? ESP_OK : ESP_ERR_TIMEOUT;
        }

        if (xTaskCheckForTimeOut(&timeout_state, &remaining) == pdTRUE &&
            !drive_command_completed(sequence)) {
            return ESP_ERR_TIMEOUT;
        }
    }

    return ESP_OK;
}

inline esp_err_t send_drive_command_and_wait(const robot_DriveCommand &command,
                                             TickType_t timeout = pdMS_TO_TICKS(5000))
{
    uint32_t sequence = 0;
    const esp_err_t send_err = send_drive_command(command, portMAX_DELAY, &sequence);
    if (send_err != ESP_OK) return send_err;
    return wait_for_drive_sequence(sequence, timeout);
}

inline esp_err_t send_pose_and_wait(const Pose &waypoint,
                                    bool relative = false,
                                    TickType_t timeout = pdMS_TO_TICKS(5000),
                                    float desired_speed_mps = DEFAULT_PATH_SPEED_MPS)
{
    uint32_t sequence = 0;
    const esp_err_t send_err = send_pose(waypoint, relative, &sequence, desired_speed_mps);
    if (send_err != ESP_OK) return send_err;
    return wait_for_drive_sequence(sequence, timeout);
}

inline esp_err_t send_pose_through(const Pose &waypoint,
                                   float pass_radius_m = 0.15f,
                                   TickType_t timeout = pdMS_TO_TICKS(5000),
                                   float desired_speed_mps = DEFAULT_PATH_SPEED_MPS)
{
    if (!std::isfinite(pass_radius_m) || pass_radius_m <= 0.0f) return ESP_ERR_INVALID_ARG;

    uint32_t sequence = 0;
    const esp_err_t send_err = send_pose(waypoint, false, &sequence, desired_speed_mps);
    if (send_err != ESP_OK) return send_err;

    const TickType_t start = xTaskGetTickCount();

    while (xTaskGetTickCount() - start < timeout) {
        if (drive_fault_active()) return ESP_FAIL;

        drive_DriveUartMessage status = drive_DriveUartMessage_init_zero;

        if (get_drive_status(&status) == ESP_OK && status.last_command_sequence == sequence &&
            (status.flags & robot_flags::DRIVE_STATUS_POSE_VALID) != 0) {
            const float dx = waypoint.x_m - status.pose.x_m;
            const float dy = waypoint.y_m - status.pose.y_m;

            if (std::hypot(dx, dy) <= pass_radius_m || drive_command_completed(sequence))
                return ESP_OK;
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }

    return ESP_ERR_TIMEOUT;
}

[[noreturn]] inline void halt_autonomous(const char *phase, esp_err_t err)
{
    ESP_LOGE("drive_pose", "%s failed: %s; stopping autonomous", phase, esp_err_to_name(err));

    xEventGroupClearBits(supervisor::g_robot_control_flags, robot_flags::CONTROL_ACTUATORS);

    robot_DriveCommand stop = robot_DriveCommand_init_zero;
    stop.which_command = robot_DriveCommand_stop_tag;
    stop.command.stop.brake = true;
    ESP_ERROR_CHECK_WITHOUT_ABORT(send_drive_command(stop));

    vTaskDelay(portMAX_DELAY);
    __builtin_unreachable();
}

inline void follow_route(std::initializer_list<Pose> route,
                         float desired_speed_mps = DEFAULT_PATH_SPEED_MPS)
{
    size_t index = 0;
    for (const Pose &pose : route) {
        const bool is_last = ++index == route.size();
        const esp_err_t err =
            is_last ? send_pose_and_wait(pose, false, pdMS_TO_TICKS(5000), desired_speed_mps)
                    : send_pose_through(pose, 0.15f, pdMS_TO_TICKS(5000), desired_speed_mps);
        if (err != ESP_OK) {
            ESP_LOGE("drive_pose",
                     "route failed at waypoint x=%.2fm, y=%.2fm, heading=%.2fdeg",
                     pose.x_m,
                     pose.y_m,
                     degrees(pose.heading_rad));
            halt_autonomous("pose route", err);
        }
    }
}
