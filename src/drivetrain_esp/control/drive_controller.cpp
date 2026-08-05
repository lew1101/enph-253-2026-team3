#include "control/drive_controller.hpp"

#include <algorithm>
#include <cinttypes>
#include <cmath>

#include "control/path.hpp"
#include "esp_err.h"
#include "esp_log.h"

#include "tasks/drive.hpp"
#include "tasks/tape_sense.hpp"

using namespace DriveTaskConfig;

using control::Drivetrain;
using control::Pose;
using control::PoseSnapshot;
using control::TapeAlignmentPhase;

namespace {
constexpr char TAG[] = "drive_controller";

const char *tape_alignment_phase_name(TapeAlignmentPhase phase)
{
    switch (phase) {
        case TapeAlignmentPhase::UNINITIALIZED:
            return "UNINITIALIZED";
        case TapeAlignmentPhase::PREPOSITION:
            return "PREPOSITION";
        case TapeAlignmentPhase::CONFIRM_CLEAR:
            return "CONFIRM_CLEAR";
        case TapeAlignmentPhase::SCAN:
            return "SCAN";
        case TapeAlignmentPhase::SCAN_PAST_TAPE:
            return "SCAN_PAST_TAPE";
        case TapeAlignmentPhase::MOVE_TO_CENTRE:
            return "MOVE_TO_CENTRE";
        case TapeAlignmentPhase::COMPLETE:
            return "COMPLETE";
        case TapeAlignmentPhase::FAILED:
            return "FAILED";
    }

    return "UNKNOWN";
}
} // namespace

Pose DriveController::_robot_to_field(const Pose &robot_pose, float field_heading_rad)
{
    float sin_h, cos_h;
    sincosf(field_heading_rad, &sin_h, &cos_h);

    return {
        .x_m = robot_pose.x_m * cos_h - robot_pose.y_m * sin_h,
        .y_m = robot_pose.x_m * sin_h + robot_pose.y_m * cos_h,
        .heading_rad = robot_pose.heading_rad,
    };
}

Pose DriveController::_field_to_robot(const Pose &field_pose, float field_heading_rad)
{
    float sin_h, cos_h;
    sincosf(field_heading_rad, &sin_h, &cos_h);

    return {
        .x_m = field_pose.x_m * cos_h + field_pose.y_m * sin_h,
        .y_m = -field_pose.x_m * sin_h + field_pose.y_m * cos_h,
        .heading_rad = field_pose.heading_rad,
    };
}

DriveController::PoseReferenceError DriveController::_get_pose_reference_error(
    const Pose &reference, const Pose &measured)
{
    const Pose field_error{
        .x_m = reference.x_m - measured.x_m,
        .y_m = reference.y_m - measured.y_m,
        .heading_rad = control::wrap_angle_pi(reference.heading_rad - measured.heading_rad),
    };

    return {
        .field_pose = field_error,
        .robot_pose = _field_to_robot(field_error, measured.heading_rad),
    };
}

void DriveController::_update_velocity_reference(Pose &reference,
                                                 const ReferenceVelocity &velocity,
                                                 const Pose &measured,
                                                 float dt_s)
{
    const PoseReferenceError error = _get_pose_reference_error(reference, measured);

    // Freeze translation or heading independently if its moving reference gets
    // too far ahead of the measured pose.
    if (std::hypot(error.field_pose.x_m, error.field_pose.y_m) <=
        VELOCITY_REFERENCE_ADVANCE_TOLERANCE_M) {
        const Pose field_step = _robot_to_field(
            {
                .x_m = velocity.vx_mps * dt_s,
                .y_m = velocity.vy_mps * dt_s,
            },
            measured.heading_rad);
        reference.x_m += field_step.x_m;
        reference.y_m += field_step.y_m;
    }

    if (fabsf(error.field_pose.heading_rad) <= VELOCITY_REFERENCE_ADVANCE_TOLERANCE_RAD) {
        reference.heading_rad =
            control::wrap_angle_pi(reference.heading_rad + velocity.heading_rate_rad_s * dt_s);
    }
}

void DriveController::_update_reference_pid_stop_state(const PoseReferenceError &error,
                                                       bool translation_requested,
                                                       bool heading_requested,
                                                       bool &position_pid_stopped,
                                                       bool &heading_pid_stopped)
{
    if (translation_requested) {
        position_pid_stopped = false;
    } else {
        const float x_tolerance =
            position_pid_stopped ? X_PID_STOP_TOLERANCE_EXIT_M : X_PID_STOP_TOLERANCE_M;
        const float y_tolerance =
            position_pid_stopped ? Y_PID_STOP_TOLERANCE_EXIT_M : Y_PID_STOP_TOLERANCE_M;
        position_pid_stopped = fabsf(error.robot_pose.x_m) <= x_tolerance &&
                               fabsf(error.robot_pose.y_m) <= y_tolerance;
    }

    if (heading_requested) {
        heading_pid_stopped = false;
    } else {
        heading_pid_stopped = fabsf(error.robot_pose.heading_rad) <=
                              (heading_pid_stopped ? HEADING_PID_STOP_TOLERANCE_EXIT_RAD
                                                   : HEADING_PID_STOP_TOLERANCE_RAD);
    }
}

void DriveController::update(Drivetrain &drivetrain,
                             const robot_DriveCommand &cmd,
                             const PoseSnapshot &pose_snapshot,
                             float dt_s)
{
    // dispatch new commands if the command tag or sequence number has changed

    const bool mode_changed = _command_tag != cmd.which_command;
    const bool new_pose_command = cmd.which_command == robot_DriveCommand_pose_tag &&
                                  (mode_changed || cmd.sequence != _previous_pose_sequence);
    const bool new_tape_alignment_command =
        cmd.which_command == robot_DriveCommand_tape_alignment_tag &&
        (mode_changed || cmd.sequence != _previous_tape_alignment_sequence);

    if (mode_changed) _handle_mode_change(cmd.which_command);
    if (new_tape_alignment_command) _begin_tape_alignment_command(cmd, pose_snapshot);
    if (new_pose_command) _begin_pose_command(cmd, pose_snapshot);

    switch (cmd.which_command) {
        case robot_DriveCommand_stop_tag:
            drivetrain.stop();
            break;

        case robot_DriveCommand_velocity_tag:
            _update_velocity(drivetrain, cmd.command.velocity, pose_snapshot, dt_s);
            break;

        case robot_DriveCommand_pose_tag:
            _update_pose(drivetrain, pose_snapshot, dt_s);
            break;

        case robot_DriveCommand_tape_alignment_tag:
            _update_tape_alignment(drivetrain, pose_snapshot, dt_s);
            break;

        case robot_DriveCommand_tape_follow_tag:
            _update_tape_follow(drivetrain, cmd.command.tape_follow, dt_s);
            break;

        default:
            drivetrain.stop();
            break;
    }
}

void DriveController::_drive_to_reference(Drivetrain &drivetrain,
                                          const PoseReferenceError &error,
                                          bool position_pid_stopped,
                                          bool heading_pid_stopped,
                                          float dt_s,
                                          float minimum_translation_command)
{
    float x_cmd = position_pid_stopped ? 0.0f : _x_pid.update(0.0f, -error.robot_pose.x_m, dt_s);
    float y_cmd = position_pid_stopped ? 0.0f : _y_pid.update(0.0f, -error.robot_pose.y_m, dt_s);
    const float heading_cmd =
        heading_pid_stopped ? 0.0f : _heading_pid.update(0.0f, -error.robot_pose.heading_rad, dt_s);

    const float translation_command = std::hypot(x_cmd, y_cmd);
    if (!position_pid_stopped && translation_command > 0.0f &&
        translation_command < minimum_translation_command) {
        const float scale = minimum_translation_command / translation_command;
        x_cmd *= scale;
        y_cmd *= scale;
    }

    drivetrain.move_vector(x_cmd, y_cmd, heading_cmd);
}

void DriveController::_reset_pose_pids()
{
    _x_pid.reset();
    _y_pid.reset();
    _heading_pid.reset();
}

void DriveController::_handle_mode_change(pb_size_t command_tag)
{
    _command_tag = command_tag;
    _fault.store(0, std::memory_order_release);

    switch (_command_tag) {
        case robot_DriveCommand_velocity_tag:
            _velocity = {};
            _reset_pose_pids();
            break;

        case robot_DriveCommand_tape_follow_tag:
            _tape_pid.reset();
            break;

        case robot_DriveCommand_stop_tag:
        case robot_DriveCommand_pose_tag:
        case robot_DriveCommand_tape_alignment_tag:
        default:
            break;
    }
}

void DriveController::_begin_tape_alignment_command(const robot_DriveCommand &cmd,
                                                    const PoseSnapshot &pose_snapshot)
{
    _fault.store(0, std::memory_order_release);
    _tape_alignment = {};
    if (cmd.command.tape_alignment.staging_direction != 0.0f) {
        _tape_alignment.config.staging_direction = cmd.command.tape_alignment.staging_direction;
    }

    _tape_alignment.reference.pose = pose_snapshot.pose;
    _previous_tape_alignment_sequence = cmd.sequence;

    ESP_LOGI(TAG,
             "tape alignment start: sequence=%" PRIu32
             " staging_direction=%.0f speed=%.2f m/s max_staging=%.3f m max_scan=%.3f m"
             " post_detection_scan=%.3f m",
             cmd.sequence,
             _tape_alignment.config.staging_direction,
             _tape_alignment.config.search_speed_mps,
             _tape_alignment.config.max_staging_distance_m,
             _tape_alignment.config.max_scan_distance_m,
             _tape_alignment.config.post_detection_scan_distance_m);

    _reset_pose_pids();
    _reached_pose.store(false, std::memory_order_release);
}

void DriveController::_begin_pose_command(const robot_DriveCommand &cmd,
                                          const PoseSnapshot &pose_snapshot)
{
    const auto &pose_command = cmd.command.pose;

    // A valid measured pose is point A for both absolute and relative paths.
    if (!pose_snapshot.valid) {
        _pose.target_valid = false;
        return;
    }

    if (pose_command.relative) {
        const Pose field_offset = _robot_to_field(
            {
                .x_m = pose_command.x_m,
                .y_m = pose_command.y_m,
                .heading_rad = pose_command.theta_rad,
            },
            pose_snapshot.pose.heading_rad);

        _pose.path_end.x_m = pose_snapshot.pose.x_m + field_offset.x_m;
        _pose.path_end.y_m = pose_snapshot.pose.y_m + field_offset.y_m;
    } else {
        _pose.path_end.x_m = pose_command.x_m;
        _pose.path_end.y_m = pose_command.y_m;
    }

    const float angle = pose_command.relative
                            ? pose_snapshot.pose.heading_rad + pose_command.theta_rad
                            : pose_command.theta_rad;
    _pose.path_end.heading_rad = control::wrap_angle_pi(angle);

    _pose.path_start = pose_snapshot.pose;
    _pose.path_distance_m = std::hypot(_pose.path_end.x_m - _pose.path_start.x_m,
                                       _pose.path_end.y_m - _pose.path_start.y_m);
    _pose.path_progress_m = 0.0f;
    if (_pose.path_distance_m > 0.0f) {
        _pose.path_unit_x = (_pose.path_end.x_m - _pose.path_start.x_m) / _pose.path_distance_m;
        _pose.path_unit_y = (_pose.path_end.y_m - _pose.path_start.y_m) / _pose.path_distance_m;
    } else {
        _pose.path_unit_x = 0.0f;
        _pose.path_unit_y = 0.0f;
    }
    _pose.path_lookahead_m = pose_command.path_lookahead_m > 0.0f
                                 ? pose_command.path_lookahead_m
                                 : DEFAULT_POSE_PATH_LOOKAHEAD_M;

    // A zero-length translation is a pure heading command, so it can use the
    // endpoint controller immediately.
    _pose.path_endpoint_active = _pose.path_distance_m <= 0.0f;
    _pose.reference.pose = _pose.path_endpoint_active ? _pose.path_end : _pose.path_start;
    _pose.target_valid = true;
    _pose.reference.position_pid_stopped = false;
    _pose.reference.heading_pid_stopped = false;
    _pose.target_reached_latched = false;
    _pose.endpoint_motion_tracking = false;
    _previous_pose_sequence = cmd.sequence;

    _reset_pose_pids();
    _reached_pose.store(false, std::memory_order_release);
}

void DriveController::_update_velocity(Drivetrain &drivetrain,
                                       const robot_VelocityCommand &velocity,
                                       const PoseSnapshot &pose_snapshot,
                                       float dt_s)
{
    if (!pose_snapshot.valid) {
        _velocity.target_valid = false;
        drivetrain.stop();
        return;
    }

    if (!_velocity.target_valid) {
        _velocity.reference.pose = pose_snapshot.pose;
        _velocity.reference.position_pid_stopped = false;
        _velocity.reference.heading_pid_stopped = false;
        _velocity.target_valid = true;
        _reset_pose_pids();
    }

    const ReferenceVelocity requested_velocity{
        .vx_mps = velocity.vx_percent / 100.0f * VELOCITY_COMMAND_MAX_TRANSLATION_MPS,
        .vy_mps = velocity.vy_percent / 100.0f * VELOCITY_COMMAND_MAX_TRANSLATION_MPS,
        .heading_rate_rad_s =
            velocity.omega_percent / 100.0f * VELOCITY_COMMAND_MAX_HEADING_RATE_RAD_S,
    };

    _update_velocity_reference(
        _velocity.reference.pose, requested_velocity, pose_snapshot.pose, dt_s);

    const PoseReferenceError error =
        _get_pose_reference_error(_velocity.reference.pose, pose_snapshot.pose);
    const bool translation_requested = velocity.vx_percent != 0.0f || velocity.vy_percent != 0.0f;
    _update_reference_pid_stop_state(error,
                                     translation_requested,
                                     velocity.omega_percent != 0.0f,
                                     _velocity.reference.position_pid_stopped,
                                     _velocity.reference.heading_pid_stopped);

    _drive_to_reference(drivetrain,
                        error,
                        _velocity.reference.position_pid_stopped,
                        _velocity.reference.heading_pid_stopped,
                        dt_s);
}

void DriveController::_update_pose(Drivetrain &drivetrain,
                                   const PoseSnapshot &pose_snapshot,
                                   float dt_s)
{
    if (!pose_snapshot.valid || !_pose.target_valid) {
        drivetrain.stop();
        return;
    }

    // Keep the reference the requested distance ahead of measured progress
    // along the path. Never move it backward if pose data jitters.
    if (!_pose.path_endpoint_active) {
        const float measured_path_progress_m = std::clamp(
            (pose_snapshot.pose.x_m - _pose.path_start.x_m) * _pose.path_unit_x +
                (pose_snapshot.pose.y_m - _pose.path_start.y_m) * _pose.path_unit_y,
            0.0f,
            _pose.path_distance_m);
        const float lookahead_progress_m =
            std::min(measured_path_progress_m + _pose.path_lookahead_m, _pose.path_distance_m);

        _pose.path_progress_m = std::max(_pose.path_progress_m, lookahead_progress_m);
        _pose.reference.pose = control::lerp_pose(
            _pose.path_start, _pose.path_end, _pose.path_progress_m / _pose.path_distance_m);
        _pose.path_endpoint_active = _pose.path_progress_m >= _pose.path_distance_m;
        _pose.endpoint_motion_tracking = false;
        _reached_pose.store(false, std::memory_order_relaxed);
    }

    const PoseReferenceError error =
        _get_pose_reference_error(_pose.reference.pose, pose_snapshot.pose);

    if (_pose.path_endpoint_active) {
        const float x_tolerance = _pose.reference.position_pid_stopped ? X_PID_STOP_TOLERANCE_EXIT_M
                                                                       : X_PID_STOP_TOLERANCE_M;
        const float y_tolerance = _pose.reference.position_pid_stopped ? Y_PID_STOP_TOLERANCE_EXIT_M
                                                                       : Y_PID_STOP_TOLERANCE_M;

        _pose.reference.position_pid_stopped = fabsf(error.robot_pose.x_m) <= x_tolerance &&
                                               fabsf(error.robot_pose.y_m) <= y_tolerance;
        _pose.reference.heading_pid_stopped =
            fabsf(error.robot_pose.heading_rad) <= (_pose.reference.heading_pid_stopped
                                                        ? HEADING_PID_STOP_TOLERANCE_EXIT_RAD
                                                        : HEADING_PID_STOP_TOLERANCE_RAD);

        const bool within_reached_bounds =
            fabsf(error.robot_pose.x_m) <= TARGET_REACHED_X_TOLERANCE_M &&
            fabsf(error.robot_pose.y_m) <= TARGET_REACHED_Y_TOLERANCE_M &&
            fabsf(error.robot_pose.heading_rad) <= TARGET_REACHED_HEADING_TOLERANCE_RAD;

        const TickType_t now = pose_snapshot.tick;
        if (!_pose.endpoint_motion_tracking) {
            _pose.endpoint_motion_reference = pose_snapshot.pose;
            _pose.endpoint_last_motion_tick = now;
            _pose.endpoint_motion_tracking = true;
        } else {
            const float translation_delta =
                std::hypot(pose_snapshot.pose.x_m - _pose.endpoint_motion_reference.x_m,
                           pose_snapshot.pose.y_m - _pose.endpoint_motion_reference.y_m);
            const float heading_delta = fabsf(control::wrap_angle_pi(
                pose_snapshot.pose.heading_rad - _pose.endpoint_motion_reference.heading_rad));

            if (translation_delta >= TARGET_SETTLED_TRANSLATION_DELTA_M ||
                heading_delta >= TARGET_SETTLED_HEADING_DELTA_RAD) {
                _pose.endpoint_motion_reference = pose_snapshot.pose;
                _pose.endpoint_last_motion_tick = now;
            }
        }

        const bool within_settle_bounds =
            std::hypot(error.field_pose.x_m, error.field_pose.y_m) <=
                TARGET_SETTLED_MAX_POSITION_ERROR_M &&
            fabsf(error.field_pose.heading_rad) <= TARGET_SETTLED_MAX_HEADING_ERROR_RAD;
        const bool stopped_long_enough =
            _pose.endpoint_motion_tracking &&
            (now - _pose.endpoint_last_motion_tick) >= pdMS_TO_TICKS(TARGET_SETTLED_TIME_MS);

        const bool reached_now =
            within_reached_bounds || (within_settle_bounds && stopped_long_enough);
        if (reached_now && !_pose.target_reached_latched) {
            _pose.target_reached_latched = true;
            _reached_pose.store(true, std::memory_order_release);
        }

        if (_pose.reference.position_pid_stopped && _pose.reference.heading_pid_stopped) {
            drivetrain.stop();
            return;
        }
    } else {
        // Intermediate references do not latch or wait for heading.
        _pose.reference.position_pid_stopped = false;
        _pose.reference.heading_pid_stopped = false;
        _pose.endpoint_motion_tracking = false;
        _reached_pose.store(false, std::memory_order_relaxed);
    }

    _drive_to_reference(drivetrain,
                        error,
                        _pose.reference.position_pid_stopped,
                        _pose.reference.heading_pid_stopped,
                        dt_s);
}

void DriveController::_update_tape_alignment(Drivetrain &drivetrain,
                                             const PoseSnapshot &pose_snapshot,
                                             float dt_s)
{
    // A failed alignment remains terminal until a new sequence resets its state.
    if (_tape_alignment.alignment.phase == control::TapeAlignmentPhase::FAILED) {
        drivetrain.stop();
        return;
    }

    TapeSnapshot tape_snapshot;
    const bool have_tape_snapshot = get_tape_snapshot(&tape_snapshot, 0);
    const bool tape_snapshot_fresh =
        have_tape_snapshot && tape_snapshot.valid &&
        (xTaskGetTickCount() - tape_snapshot.tick) <= pdMS_TO_TICKS(TAPE_SNAPSHOT_TIMEOUT_MS);

    if (!tape_snapshot_fresh) {
        ESP_LOGW(TAG, "failed to get tape snapshot for alignment");
        drivetrain.stop();
        _reached_pose.store(false, std::memory_order_relaxed);
        _fault.store(static_cast<uint32_t>(ESP_ERR_TIMEOUT), std::memory_order_release);
        _tape_alignment.alignment.phase = control::TapeAlignmentPhase::FAILED;
        return;
    }

    const control::TapeAlignmentOutput alignment =
        control::update_tape_alignment(_tape_alignment.alignment,
                                       _tape_alignment.config,
                                       tape_snapshot.tape_fl,
                                       tape_snapshot.tape_fr,
                                       tape_snapshot.tick,
                                       pose_snapshot);

    const auto &capture = _tape_alignment.alignment.crossing_capture;
    if (capture.sensor_a.tape_entry_position_m && !_tape_alignment.logged_sensor_a_edge) {
        ESP_LOGI(TAG,
                 "tape alignment captured L1 white-to-black edge at %.3f m",
                 *capture.sensor_a.tape_entry_position_m);
        _tape_alignment.logged_sensor_a_edge = true;
    }
    if (capture.sensor_b.tape_entry_position_m && !_tape_alignment.logged_sensor_b_edge) {
        ESP_LOGI(TAG,
                 "tape alignment captured L2 white-to-black edge at %.3f m",
                 *capture.sensor_b.tape_entry_position_m);
        _tape_alignment.logged_sensor_b_edge = true;
    }

    const bool phase_changed =
        !_tape_alignment.have_phase || alignment.phase != _tape_alignment.previous_phase;
    if (phase_changed) {
        const esp_log_level_t level =
            alignment.phase == control::TapeAlignmentPhase::FAILED ? ESP_LOG_ERROR : ESP_LOG_INFO;
        esp_log_write(level,
                      TAG,
                      "tape alignment phase: %s -> %s; l1=%d l2=%d pose=(%.3f, %.3f, %.1f deg)\n",
                      _tape_alignment.have_phase
                          ? tape_alignment_phase_name(_tape_alignment.previous_phase)
                          : "START",
                      tape_alignment_phase_name(alignment.phase),
                      tape_snapshot.tape_l1,
                      tape_snapshot.tape_l2,
                      pose_snapshot.pose.x_m,
                      pose_snapshot.pose.y_m,
                      pose_snapshot.pose.heading_rad * 180.0f / static_cast<float>(M_PI));

        if (alignment.pose_target) {
            ESP_LOGI(TAG,
                     "tape alignment target: (%.3f, %.3f, %.1f deg)",
                     alignment.pose_target->x_m,
                     alignment.pose_target->y_m,
                     alignment.pose_target->heading_rad * 180.0f / static_cast<float>(M_PI));
        }

        _tape_alignment.previous_phase = alignment.phase;
        _tape_alignment.have_phase = true;
    }

    const bool action_changed =
        !_tape_alignment.have_action || alignment.action != _tape_alignment.previous_action;
    if (action_changed) {
        _tape_alignment.reference.position_pid_stopped = false;
        _tape_alignment.reference.heading_pid_stopped = false;
        _reset_pose_pids();

        if (alignment.action == control::TapeAlignmentAction::HOLD ||
            alignment.action == control::TapeAlignmentAction::VELOCITY) {
            _tape_alignment.reference.pose = pose_snapshot.pose;
        } else if (alignment.action == control::TapeAlignmentAction::POSE &&
                   alignment.pose_target) {
            _tape_alignment.reference.pose = *alignment.pose_target;
        }

        _tape_alignment.previous_action = alignment.action;
        _tape_alignment.have_action = true;
    }

    switch (alignment.action) {
        case control::TapeAlignmentAction::VELOCITY: {
            const ReferenceVelocity requested_velocity{
                .vx_mps = alignment.velocity.vx_mps,
                .vy_mps = alignment.velocity.vy_mps,
                .heading_rate_rad_s = alignment.velocity.heading_rate_rad_s,
            };
            _update_velocity_reference(
                _tape_alignment.reference.pose, requested_velocity, pose_snapshot.pose, dt_s);

            const PoseReferenceError error =
                _get_pose_reference_error(_tape_alignment.reference.pose, pose_snapshot.pose);
            const bool translation_requested =
                requested_velocity.vx_mps != 0.0f || requested_velocity.vy_mps != 0.0f;
            _update_reference_pid_stop_state(error,
                                             translation_requested,
                                             requested_velocity.heading_rate_rad_s != 0.0f,
                                             _tape_alignment.reference.position_pid_stopped,
                                             _tape_alignment.reference.heading_pid_stopped);
            _drive_to_reference(drivetrain,
                                error,
                                _tape_alignment.reference.position_pid_stopped,
                                _tape_alignment.reference.heading_pid_stopped,
                                dt_s);
            _reached_pose.store(false, std::memory_order_relaxed);
            break;
        }

        case control::TapeAlignmentAction::HOLD:
        case control::TapeAlignmentAction::POSE: {
            if (alignment.action == control::TapeAlignmentAction::POSE && !alignment.pose_target) {
                drivetrain.stop();
                _fault.store(static_cast<uint32_t>(ESP_FAIL), std::memory_order_release);
                break;
            }

            const PoseReferenceError error =
                _get_pose_reference_error(_tape_alignment.reference.pose, pose_snapshot.pose);
            const bool actively_centering = alignment.action == control::TapeAlignmentAction::POSE;
            _update_reference_pid_stop_state(error,
                                             actively_centering,
                                             actively_centering,
                                             _tape_alignment.reference.position_pid_stopped,
                                             _tape_alignment.reference.heading_pid_stopped);
            _drive_to_reference(drivetrain,
                                error,
                                _tape_alignment.reference.position_pid_stopped,
                                _tape_alignment.reference.heading_pid_stopped,
                                dt_s,
                                actively_centering ? TAPE_CENTER_MIN_TRANSLATION_COMMAND_PERCENT
                                                   : 0.0f);
            _reached_pose.store(false, std::memory_order_relaxed);
            break;
        }

        case control::TapeAlignmentAction::COMPLETE:
            drivetrain.stop();
            if (action_changed) {
                _reached_pose.store(true, std::memory_order_release);
            }
            break;

        case control::TapeAlignmentAction::FAILED:
            drivetrain.stop();
            _reached_pose.store(false, std::memory_order_release);
            _fault.store(static_cast<uint32_t>(ESP_FAIL), std::memory_order_release);
            break;
    }
}

void DriveController::_update_tape_follow(Drivetrain &drivetrain,
                                          const robot_TapeFollowCommand &command,
                                          float dt_s)
{
    static bool have_warning_tick = false;
    static TickType_t last_warning_tick = 0;

    TapeSnapshot tape_snapshot;
    const bool have_tape_snapshot = get_tape_snapshot(&tape_snapshot, 0);
    const TickType_t now = xTaskGetTickCount();
    const bool tape_snapshot_fresh =
        have_tape_snapshot && tape_snapshot.valid &&
        (now - tape_snapshot.tick) <= pdMS_TO_TICKS(TAPE_SNAPSHOT_TIMEOUT_MS);

    if (!tape_snapshot_fresh) {
        if (!have_warning_tick ||
            (now - last_warning_tick) >= pdMS_TO_TICKS(TAPE_SNAPSHOT_WARNING_PERIOD_MS)) {
            ESP_LOGW(TAG, "invalid or stale tape snapshot");
            last_warning_tick = now;
            have_warning_tick = true;
        }
        drivetrain.stop();
        return;
    }

    have_warning_tick = false;

    const bool is_reversed = command.forward_speed_percent < 0.0f;
    const float correction = is_reversed ? -_tape_pid.update(0.0f, tape_snapshot.back_err, dt_s)
                                         : _tape_pid.update(0.0f, tape_snapshot.front_err, dt_s);
    const float left_speed = command.forward_speed_percent + correction;
    const float right_speed = command.forward_speed_percent - correction;

    if (is_reversed) {
        drivetrain.move_front(left_speed, right_speed);
    } else {
        drivetrain.move_rear(left_speed, right_speed);
    }
}
