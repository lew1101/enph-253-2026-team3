#pragma once

#include <cstdint>
#include <optional>

#include "control/pose_estimator.hpp"

/*
 * 1. Sweep/strafe left
 * 2. Stop briefly and confirm both sensors are off the tape
 * 3. Sweep/strafe right across the tape
 * 4. Record each sensor's white-to-black transition position
 * 5. Calculate the tape center from those two edge positions
 * 6. Continue past the detected edge by a configured distance
 * 7. Move back to the calculated center
 */

namespace control {

// state machine phases for tape alignment
enum class TapeAlignmentPhase {
    UNINITIALIZED,  // not yet started
    PREPOSITION,    // moving to a known clear side of the tape
    CONFIRM_CLEAR,  // confirming that both sensors are off the tape
    SCAN,           // moving across the tape to capture the tape edges
    SCAN_PAST_TAPE, // continuing in the scan direction after both edges are captured
    MOVE_TO_CENTRE, // moving to the calculated tape center
    COMPLETE,       // reached the target center position
    FAILED,         // failed to complete the procedure
};

enum class TapeAlignmentAction {
    HOLD,     // maintain current position
    VELOCITY, // move at a specified velocity
    POSE,     // move to a specified pose
    COMPLETE, // reached the target center position
    FAILED,   // failed to complete the procedure
};

enum class TapeAlignmentFailure {
    NONE,
    INVALID_POSE,
    INVALID_CONFIG,
    TIMEOUT,
    MAX_STAGING_DISTANCE,
    MAX_SCAN_DISTANCE,
    MISSING_CENTER_TARGET,
};

struct TapeAlignmentVelocity {
    float vx_mps = 0.0f;
    float vy_mps = 0.0f;
    float heading_rate_rad_s = 0.0f;
};

/*
 * staging_direction: direction used to get onto a known clear side of the tape
 * search_speed_mps: strafe speed during staging and scanning
 * staging_distance_m: minimum distance to move before accepting that the robot is clear
 * max_staging_distance_m: fail if it cannot get clear within this distance
 * max_scan_distance_m: fail if it cannot cross the whole tape within this distance
 * post_detection_scan_distance_m: continue this far after capturing both sensor edges
 * center_tolerance_m: how close the robot must get to the target
 * heading_tolerance_rad: allowed heading error
 * timeout_ms: maximum duration of the entire procedure
 */

struct TapeAlignmentConfig {
    float staging_direction = -1.0f;  // +ve is to the right of the robot, -ve is to the left
    float search_speed_mps = 0.16f;   // strafe speed during staging and scanning
    float staging_distance_m = 0.09f; // minimum distance before accepting that the robot is clear
    float max_staging_distance_m = 0.15f; // fail if it cannot get clear within this distance
    float max_scan_distance_m = 0.2f;    // fail if it doesn't cross tape within this distance
    float post_detection_scan_distance_m = 0.07f; // travel farther before returning to center
    float center_tolerance_m = 0.005f;    // how close the robot must get to the target
    float heading_tolerance_rad = 1.5f * static_cast<float>(M_PI) / 180.0f; // allowed heading error

    uint8_t clear_debounce_samples = 3; // consecutive all-white samples before scanning
    uint8_t edge_debounce_samples = 1;  // white-to-black samples required to capture an edge
    uint32_t timeout_ms = 8000;         // maximum duration of the entire procedure
};

struct TapeEdgeCapture {
    bool stable_state = false;         // definite state (passed debounce)
    bool candidate_state = false;      // possible state
    uint8_t candidate_samples = 0;     // number of consecutive samples in the candidate state
    float candidate_position_m = 0.0f; // position when the candidate state was first detected

    std::optional<float> tape_entry_position_m; // first confirmed white-to-black transition
};

struct TapeCenterCapture {
    float strafe_heading_rad = 0.0f; // heading of the robot when the tape was crossed

    TapeEdgeCapture sensor_a; // populated when cross tape with sensor A
    TapeEdgeCapture sensor_b; // populated when cross tape with sensor B
};

struct TapeAlignmentState {
    TapeAlignmentPhase phase = TapeAlignmentPhase::UNINITIALIZED;
    TickType_t start_tick = 0;
    TickType_t last_tape_tick = 0;
    bool have_tape_tick = false;
    uint8_t clear_samples = 0;

    float strafe_heading_rad = 0.0f;
    Pose field_strafe_axis{};
    float start_strafe_position_m = 0.0f;
    float scan_start_position_m = 0.0f;
    float post_detection_start_position_m = 0.0f;

    TapeCenterCapture crossing_capture{};
    std::optional<Pose> center_target;
};

struct TapeAlignmentOutput {
    TapeAlignmentPhase phase =
        TapeAlignmentPhase::UNINITIALIZED;                  // current phase of the state machine
    TapeAlignmentAction action = TapeAlignmentAction::HOLD; // what the drive task should do next

    TapeAlignmentVelocity velocity{}; // desired velocity if action is VELOCITY
    std::optional<Pose>
        pose_target; // desired pose if action is POSE, or the calculated center if action is COMPLETE
    TapeAlignmentFailure failure = TapeAlignmentFailure::NONE;
};

/*
 * Call once per drive iteration. Duplicate tape
 * snapshot ticks are ignored for debouncing.
 */
TapeAlignmentOutput update_tape_alignment(TapeAlignmentState &state,
                                          const TapeAlignmentConfig &cfg,
                                          bool sensor_a_sees_tape,
                                          bool sensor_b_sees_tape,
                                          TickType_t tape_tick,
                                          const PoseSnapshot &pose_snapshot);

} // namespace control
