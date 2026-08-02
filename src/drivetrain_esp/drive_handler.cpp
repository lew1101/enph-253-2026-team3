#include <algorithm>
#include <cmath>

#include "shared/robot_flags.hpp"
#include "drive_handler.hpp"
#include "freertos/idf_additions.h"
#include "portmacro.h"
#include "tasks/drive.hpp"

DriveMessageHandler::DriveMessageHandler()
{
    _mutex = xSemaphoreCreateMutex();
    configASSERT(_mutex != nullptr);
}

bool DriveMessageHandler::_is_newer_sequence(uint32_t sequence, uint32_t previous)
{
    return static_cast<int32_t>(sequence - previous) > 0;
}

void DriveMessageHandler::_stop()
{
    robot_DriveCommand command = robot_DriveCommand_init_zero;
    command.which_command = robot_DriveCommand_stop_tag;

    if (send_drive_cmd(command) != ESP_OK) {
        xSemaphoreTake(_mutex, portMAX_DELAY);
        _fault = 1;
        xSemaphoreGive(_mutex);
    };
}

esp_err_t DriveMessageHandler::_apply_command(robot_DriveCommand message)
{
    xSemaphoreTake(_mutex, portMAX_DELAY);
    auto have_sequence = _have_sequence;
    auto last_sequence = _last_sequence;
    auto estop = _estop;
    auto drive_enabled = _drive_enabled;
    auto tape_enabled = _tape_enabled;
    xSemaphoreGive(_mutex);

    if (have_sequence) {
        if (message.sequence == last_sequence) return ESP_OK; // idempotent retry
        if (!_is_newer_sequence(message.sequence, last_sequence))
            return ESP_ERR_INVALID_STATE; // invalid command??
    }

    // if estop is pressed, disallow applying any other command
    if (estop && message.which_command != robot_DriveCommand_stop_tag) return ESP_ERR_INVALID_STATE;

    // disallow driving if stop command active
    if (!drive_enabled && message.which_command != robot_DriveCommand_stop_tag)
        return ESP_ERR_INVALID_STATE;

    const bool tape_command = message.which_command == robot_DriveCommand_tape_follow_tag ||
                              message.which_command == robot_DriveCommand_tape_alignment_tag;
    if (!tape_enabled && tape_command)
        return ESP_ERR_INVALID_STATE;

    switch (message.which_command) {
        case robot_DriveCommand_velocity_tag: {
            auto &m = message.command.velocity;

            if (!std::isfinite(m.vx_percent) || !std::isfinite(m.vy_percent) ||
                !std::isfinite(m.omega_percent))
                return ESP_ERR_INVALID_ARG;

            m.vx_percent = std::clamp(m.vx_percent, -100.0f, 100.0f);
            m.vy_percent = std::clamp(m.vy_percent, -100.0f, 100.0f);
            m.omega_percent = std::clamp(m.omega_percent, -100.0f, 100.0f);
            break;
        }
        case robot_DriveCommand_tape_follow_tag: {
            auto &m = message.command.tape_follow;

            if (!std::isfinite(m.forward_speed_percent)) return ESP_ERR_INVALID_ARG;
            m.forward_speed_percent = std::clamp(m.forward_speed_percent, -100.0f, 100.0f);
            break;
        }
        case robot_DriveCommand_pose_tag: {
            auto &m = message.command.pose;
            if (!std::isfinite(m.x_m) || !std::isfinite(m.y_m) ||
                !std::isfinite(m.theta_rad) || !std::isfinite(m.desired_speed_mps) ||
                m.desired_speed_mps < 0.0f)
                return ESP_ERR_INVALID_ARG;
            break;
        }
        case robot_DriveCommand_tape_alignment_tag:
            if (!std::isfinite(message.command.tape_alignment.staging_direction))
                return ESP_ERR_INVALID_ARG;
            break;
        case robot_DriveCommand_stop_tag:
            break;
        default:
            return ESP_ERR_INVALID_ARG;
    }

    // Prevent a newly accepted sequence from inheriting the completion latch
    // belonging to the previous pose/alignment command.
    clear_reached_pose();
    const esp_err_t err = send_drive_cmd(message);
    if (err != ESP_OK) return err;

    xSemaphoreTake(_mutex, portMAX_DELAY);
    _last_sequence = message.sequence;
    _have_sequence = true;
    _tape_command_active = tape_command;
    xSemaphoreGive(_mutex);

    return ESP_OK;
}

void DriveMessageHandler::on_link_disconnected()
{
    xSemaphoreTake(_mutex, portMAX_DELAY);
    _have_sequence = false;
    xSemaphoreGive(_mutex);

    _stop();
}

esp_err_t DriveMessageHandler::handle(const robot_RobotUartMessage &message)
{
    using robot_flags::has_flag;

    esp_err_t err = ESP_OK;

    switch (message.which_payload) {
        case robot_RobotUartMessage_drive_command_tag:
            // remember to set state before sending a particular drive command
            err = _apply_command(message.payload.drive_command);
            break;

        case robot_RobotUartMessage_state_tag: {
            const uint32_t flags = message.payload.state.control_flags;
            const uint32_t session_id = message.payload.state.session_id;
            const bool drive_enabled = has_flag(flags, robot_flags::CONTROL_DRIVE_ENABLED);
            const bool tape_enabled = has_flag(flags, robot_flags::CONTROL_TAPE_ENABLED);
            const bool estop_active = has_flag(flags, robot_flags::CONTROL_ESTOP_ACTIVE);
            const bool clear_drive_fault = has_flag(flags, robot_flags::CONTROL_CLEAR_DRIVE_FAULT);

            xSemaphoreTake(_mutex, portMAX_DELAY);
            if (!_have_session || session_id != _session_id) {
                _session_id = session_id;
                _have_session = true;
                _last_sequence = 0;
                _have_sequence = false;
            }

            const bool must_stop = (!drive_enabled && _drive_enabled) ||
                                   (!tape_enabled && _tape_command_active) ||
                                   (estop_active && !_estop);

            _drive_enabled = drive_enabled;
            _tape_enabled = tape_enabled;
            _estop = estop_active;

            if (clear_drive_fault) _fault = 0;
            if (must_stop) _tape_command_active = false;
            xSemaphoreGive(_mutex);

            if (must_stop) _stop();
            break;
        }
        default:
            err = ESP_ERR_INVALID_ARG;
    }

    if (err != ESP_OK) {
        xSemaphoreTake(_mutex, portMAX_DELAY);
        _fault = static_cast<uint32_t>(err);
        xSemaphoreGive(_mutex);
    }
    return err;
}

drive_DriveUartMessage DriveMessageHandler::make_status(bool connected, uint32_t uptime) const
{
    control::PoseSnapshot pose_snapshot{};
    const bool have_pose = get_pose(&pose_snapshot) == ESP_OK;

    xSemaphoreTake(_mutex, portMAX_DELAY);
    auto last_sequence = _last_sequence;
    auto fault = _fault;
    auto estop = _estop;
    xSemaphoreGive(_mutex);

    drive_DriveUartMessage status = drive_DriveUartMessage_init_zero;
    status.last_command_sequence = last_sequence;
    const uint32_t drive_task_fault = get_drive_task_fault();
    status.fault = fault != 0 ? fault : drive_task_fault;
    status.uptime_ms = uptime;

    if (connected) {
        status.flags |= robot_flags::DRIVE_STATUS_LINK_CONNECTED;
    }
    if (have_pose && pose_snapshot.valid) {
        status.flags |= robot_flags::DRIVE_STATUS_POSE_VALID;
    }
    if (have_pose && pose_snapshot.valid && reached_pose()) {
        status.flags |= robot_flags::DRIVE_STATUS_TARGET_REACHED;
    }
    if (estop) {
        status.flags |= robot_flags::DRIVE_STATUS_ESTOP_LATCHED;
    }
    if (have_pose) {
        status.pose.x_m = pose_snapshot.pose.x_m;
        status.pose.y_m = pose_snapshot.pose.y_m;
        status.pose.heading_rad = pose_snapshot.pose.heading_rad;
        status.pose.tick = static_cast<uint32_t>(pose_snapshot.tick);
    }

    return status;
}
