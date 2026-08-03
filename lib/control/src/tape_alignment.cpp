#include <Arduino.h>
#include "control/tape_alignment.hpp"

#include <cmath>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"

namespace control {
namespace {

/*
 * Transforms a robot pose to the field coordinate system.
 */
inline Pose _robot_to_field(const Pose &robot_pose, float field_heading_rad)
{
    float sin_h, cos_h;
    sincosf(field_heading_rad, &sin_h, &cos_h);

    return {
        .x_m = robot_pose.x_m * cos_h - robot_pose.y_m * sin_h,
        .y_m = robot_pose.x_m * sin_h + robot_pose.y_m * cos_h,
        .heading_rad = robot_pose.heading_rad,
    };
}

/**
 * Projects a pose onto the strafe axis to get a 1D position along that axis.
 */
inline float _project_onto_strafe_axis(const Pose &field_strafe_axis, const Pose &pose)
{
    return pose.x_m * field_strafe_axis.x_m + pose.y_m * field_strafe_axis.y_m;
}

/**
 * Updates the capture state for a tape edge based on sensor readings and position.
 */
void _update_tape_edge_capture(TapeEdgeCapture &capture,
                               bool sees_tape,
                               float strafe_position_m,
                               uint8_t tape_alignment_debounce_samples)
{
    if (sees_tape == capture.stable_state) {
        // current state matches stable state
        capture.candidate_samples = 0;
        return;
    }

    if (capture.candidate_samples == 0 || sees_tape != capture.candidate_state) {
        // new candidate state detected, reset the sample count
        capture.candidate_state = sees_tape;
        capture.candidate_position_m = strafe_position_m;
        capture.candidate_samples = 1;
    } else {
        ++capture.candidate_samples;
    }

    if (capture.candidate_samples < tape_alignment_debounce_samples) return;

    capture.stable_state = capture.candidate_state;
    capture.candidate_samples = 0;

    // Record the first sample in the confirmed transition, avoiding debounce
    // travel from shifting the measured crossing.
    if (capture.stable_state && !capture.tape_entry_position_m)
        capture.tape_entry_position_m = capture.candidate_position_m;
}

std::optional<float> _update_tape_center_capture(TapeCenterCapture &capture,
                                                 bool sensor_a_sees_tape,
                                                 bool sensor_b_sees_tape,
                                                 const Pose &measured_pose,
                                                 uint8_t debounce_samples)
{
    // vector along strafe axis in field coords
    const Pose field_strafe_axis = _robot_to_field({.x_m = 1.0f}, capture.strafe_heading_rad);

    // project the measured pose onto the strafe axis to get a 1D position along that axis
    const float strafe_position_m = _project_onto_strafe_axis(field_strafe_axis, measured_pose);

    _update_tape_edge_capture(
        capture.sensor_a, sensor_a_sees_tape, strafe_position_m, debounce_samples);
    _update_tape_edge_capture(
        capture.sensor_b, sensor_b_sees_tape, strafe_position_m, debounce_samples);

    const auto &a = capture.sensor_a;
    const auto &b = capture.sensor_b;

    if (!a.tape_entry_position_m || !b.tape_entry_position_m) {
        // not enough information to calculate the center yet
        return std::nullopt;
    }

    // Both sensors crossed the same tape edge. The robot is centered when its
    // strafe position is halfway between those two crossing positions.
    return (*a.tape_entry_position_m + *b.tape_entry_position_m) / 2.0f;
}

TapeAlignmentOutput _failed(TapeAlignmentState &state, TapeAlignmentFailure failure)
{
    state.phase = TapeAlignmentPhase::FAILED;
    log_i("failed");
    return {
        .phase = state.phase,
        .action = TapeAlignmentAction::FAILED,
        .pose_target = state.center_target,
        .failure = failure,
    };
}

} // namespace

TapeAlignmentOutput update_tape_alignment(TapeAlignmentState &state,
                                          const TapeAlignmentConfig &cfg,
                                          bool sensor_a_sees_tape,
                                          bool sensor_b_sees_tape,
                                          TickType_t tape_tick,
                                          const PoseSnapshot &pose_snapshot)
{
    if (!pose_snapshot.valid) return _failed(state, TapeAlignmentFailure::INVALID_POSE);

    log_i("a=%d, b=%d", sensor_a_sees_tape, sensor_b_sees_tape);

    const Pose &pose = pose_snapshot.pose;
    const bool new_tape_sample = !state.have_tape_tick || tape_tick != state.last_tape_tick;
    if (new_tape_sample) {
        state.last_tape_tick = tape_tick;
        state.have_tape_tick = true;
    }

    if (state.phase == TapeAlignmentPhase::UNINITIALIZED) {
        // validate config parameters before starting the procedure
        const bool invalid_config =
            !std::isfinite(cfg.staging_direction) || !std::isfinite(cfg.search_speed_mps) ||
            cfg.staging_direction == 0.0f || cfg.search_speed_mps <= 0.0f ||
            cfg.staging_distance_m <= 0.0f || cfg.max_staging_distance_m < cfg.staging_distance_m ||
            cfg.max_scan_distance_m <= 0.0f ||
            !std::isfinite(cfg.post_detection_scan_distance_m) ||
            cfg.post_detection_scan_distance_m < 0.0f || cfg.clear_debounce_samples == 0 ||
            cfg.edge_debounce_samples == 0;

        if (invalid_config) return _failed(state, TapeAlignmentFailure::INVALID_CONFIG);

        state.start_tick = pose_snapshot.tick;
        state.strafe_heading_rad = pose.heading_rad;
        state.field_strafe_axis = _robot_to_field({.x_m = 1.0f}, state.strafe_heading_rad);
        state.start_strafe_position_m = _project_onto_strafe_axis(state.field_strafe_axis, pose);

        // Both sensors seeing black is not proof that the robot is centered.
        // Always stage to a confirmed all-white side before scanning edges.
        state.phase = TapeAlignmentPhase::PREPOSITION;
    }

    if ((pose_snapshot.tick - state.start_tick) >= pdMS_TO_TICKS(cfg.timeout_ms))
        return _failed(state, TapeAlignmentFailure::TIMEOUT);

    // +ve is to the right of the robot, -ve is to the left. The staging direction is
    const float staging_direction = cfg.staging_direction > 0.0f ? 1.0f : -1.0f;
    const float strafe_position_m = _project_onto_strafe_axis(state.field_strafe_axis, pose);

    // current strafe axis position along the field strafe axis, +ve is to the right of the robot,
    // -ve is to the left

    switch (state.phase) {
        case TapeAlignmentPhase::PREPOSITION: {
            // move in staging direction until both sensors off tape.
            const float staging_travel_m =
                (strafe_position_m - state.start_strafe_position_m) * staging_direction;

            if (staging_travel_m >= cfg.staging_distance_m && new_tape_sample &&
                !sensor_a_sees_tape && !sensor_b_sees_tape) {
                // travelled for staging distance and both sensors are off tape, start confirming clear
                state.clear_samples = 1;
                state.phase = TapeAlignmentPhase::CONFIRM_CLEAR;

                return {
                    .phase = state.phase,
                    .action = TapeAlignmentAction::HOLD,
                };
            }

            // if we have travelled too far, fail
            if (staging_travel_m >= cfg.max_staging_distance_m)
                return _failed(state, TapeAlignmentFailure::MAX_STAGING_DISTANCE);

            // not yet stage far enough, keep moving in staging direction
            return {
                .phase = state.phase,
                .action = TapeAlignmentAction::VELOCITY,
                .velocity = {.vx_mps = staging_direction * cfg.search_speed_mps},
            };
        }

        case TapeAlignmentPhase::CONFIRM_CLEAR:
            if (new_tape_sample) {
                // what the hell, wasn't really clear, go back to prepositioning
                if (sensor_a_sees_tape || sensor_b_sees_tape) {
                    state.clear_samples = 0;
                    state.phase = TapeAlignmentPhase::PREPOSITION;

                    // keep moving in staging direction until both sensors off tape.
                    return {
                        .phase = state.phase,
                        .action = TapeAlignmentAction::VELOCITY,
                        .velocity = {.vx_mps = staging_direction * cfg.search_speed_mps},
                    };
                }

                // clear for enough consecutive samples, start scanning
                if (++state.clear_samples >= cfg.clear_debounce_samples) {
                    state.crossing_capture = {};
                    state.crossing_capture.strafe_heading_rad = state.strafe_heading_rad;
                    state.scan_start_position_m = strafe_position_m;
                    state.phase = TapeAlignmentPhase::SCAN;
                }
            }

            // if we are still confirming clear, hold position until we have enough consecutive samples
            // otherwise, transition to scanning phase
            return {
                .phase = state.phase,
                .action = state.phase == TapeAlignmentPhase::SCAN ? TapeAlignmentAction::VELOCITY
                                                                  : TapeAlignmentAction::HOLD,
                .velocity = {.vx_mps = state.phase == TapeAlignmentPhase::SCAN
                                           ? -staging_direction * cfg.search_speed_mps
                                           : 0.0f},
            };

        case TapeAlignmentPhase::SCAN: {
            // scan in the opposite direction of staging until we find the tape center or exceed max scan distance
            const float scan_direction = -staging_direction;
            const float scan_travel_m =
                (strafe_position_m - state.scan_start_position_m) * scan_direction;

            std::optional<float> center_position_m;
            if (new_tape_sample) {
                center_position_m =
                    _update_tape_center_capture(state.crossing_capture,
                                                sensor_a_sees_tape,
                                                sensor_b_sees_tape,
                                                pose,
                                                cfg.edge_debounce_samples);
            }

            if (center_position_m) {
                // we found the tape centre, yay, now next step move to centre
                const float correction_m = *center_position_m - strafe_position_m;

                state.center_target = Pose{
                    .x_m = pose.x_m + correction_m * state.field_strafe_axis.x_m,
                    .y_m = pose.y_m + correction_m * state.field_strafe_axis.y_m,
                    .heading_rad = state.strafe_heading_rad,
                };
                state.post_detection_start_position_m = strafe_position_m;
                state.phase = TapeAlignmentPhase::SCAN_PAST_TAPE;

                return {
                    .phase = state.phase,
                    .action = TapeAlignmentAction::VELOCITY,
                    .velocity = {.vx_mps = scan_direction * cfg.search_speed_mps},
                };
            }

            // scanned too far and still haven't found the tape centre, fail.
            if (scan_travel_m >= cfg.max_scan_distance_m)
                return _failed(state, TapeAlignmentFailure::MAX_SCAN_DISTANCE);

            // not found tape yet, keep scanning.
            return {
                .phase = state.phase,
                .action = TapeAlignmentAction::VELOCITY,
                .velocity = {.vx_mps = scan_direction * cfg.search_speed_mps},
            };
        }

        case TapeAlignmentPhase::SCAN_PAST_TAPE: {
            const float scan_direction = -staging_direction;
            const float travel_m =
                (strafe_position_m - state.post_detection_start_position_m) * scan_direction;

            if (travel_m >= cfg.post_detection_scan_distance_m) {
                if (!state.center_target)
                    return _failed(state, TapeAlignmentFailure::MISSING_CENTER_TARGET);

                state.phase = TapeAlignmentPhase::MOVE_TO_CENTRE;
                return {
                    .phase = state.phase,
                    .action = TapeAlignmentAction::POSE,
                    .pose_target = state.center_target,
                };
            }

            return {
                .phase = state.phase,
                .action = TapeAlignmentAction::VELOCITY,
                .velocity = {.vx_mps = scan_direction * cfg.search_speed_mps},
            };
        }

        case TapeAlignmentPhase::MOVE_TO_CENTRE: {
            if (!state.center_target)
                return _failed(state, TapeAlignmentFailure::MISSING_CENTER_TARGET);

            // find relative error between measured pose and target pose
            const float position_error_m = std::hypot(state.center_target->x_m - pose.x_m,
                                                      state.center_target->y_m - pose.y_m);
            const float heading_error_rad =
                fabsf(wrap_angle_pi(state.center_target->heading_rad - pose.heading_rad));

            if (position_error_m <= cfg.center_tolerance_m &&
                heading_error_rad <= cfg.heading_tolerance_rad) {
                // pose within tolerance on tape, yay we done
                state.phase = TapeAlignmentPhase::COMPLETE;

                return {
                    .phase = state.phase,
                    .action = TapeAlignmentAction::COMPLETE,
                    .pose_target = state.center_target,
                };
            }

            // not yet within tolerance, keep moving cuh
            return {
                .phase = state.phase,
                .action = TapeAlignmentAction::POSE,
                .pose_target = state.center_target,
            };
        }

        case TapeAlignmentPhase::COMPLETE:
            //we was done, so just keep doing the same thing
            return {
                .phase = state.phase,
                .action = TapeAlignmentAction::COMPLETE,
                .pose_target = state.center_target,
            };

        case TapeAlignmentPhase::FAILED:
            return _failed(state, TapeAlignmentFailure::NONE);

        case TapeAlignmentPhase::UNINITIALIZED:
            break;
    }

    return _failed(state, TapeAlignmentFailure::INVALID_CONFIG);
}

} // namespace control
