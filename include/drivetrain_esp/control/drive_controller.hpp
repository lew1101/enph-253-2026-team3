#pragma once

#include <atomic>
#include <cstdint>

#include "control/drivetrain.hpp"
#include "control/pid.hpp"
#include "control/pose_estimator.hpp"
#include "control/tape_alignment.hpp"
#include "control/tape_pid.hpp"

#include "drive.pb.h"

class DriveController {
  public:
    DriveController() = default;

    void update(control::Drivetrain &drivetrain,
                const robot_DriveCommand &cmd,
                const control::PoseSnapshot &pose_snapshot,
                float dt_s);

    inline bool reached_pose() const { return _reached_pose.load(std::memory_order_relaxed); }
    inline void clear_reached_pose() { _reached_pose.store(false, std::memory_order_release); }
    inline uint32_t fault() const { return _fault.load(std::memory_order_acquire); }

  private:
    struct ReferenceVelocity {
        float vx_mps = 0.0f;
        float vy_mps = 0.0f;
        float heading_rate_rad_s = 0.0f;
    };

    struct PoseReferenceError {
        control::Pose field_pose{};
        control::Pose robot_pose{};
    };

    struct ReferenceState {
        control::Pose pose{};
        bool position_pid_stopped = false;
        bool heading_pid_stopped = false;
    };

    struct VelocityState {
        ReferenceState reference{};
        bool target_valid = false;
    };

    struct PoseState {
        ReferenceState reference{};
        control::Pose path_start{};
        control::Pose path_end{};

        float path_distance_m = 0.0f;
        float path_progress_m = 0.0f;
        float path_speed_mps = 0.0f;

        bool path_endpoint_active = false;
        bool target_valid = false;
        bool target_reached_latched = false;

        control::Pose endpoint_motion_reference{};
        TickType_t endpoint_last_motion_tick = 0;
        bool endpoint_motion_tracking = false;
    };

    struct TapeAlignmentDriveState {
        control::TapeAlignmentState alignment{};
        control::TapeAlignmentConfig config{};
        control::TapeAlignmentAction previous_action = control::TapeAlignmentAction::FAILED;
        control::TapeAlignmentPhase previous_phase = control::TapeAlignmentPhase::UNINITIALIZED;
        ReferenceState reference{};
        bool have_action = false;
        bool have_phase = false;
        bool logged_sensor_a_edge = false;
        bool logged_sensor_b_edge = false;
    };

    static control::Pose _robot_to_field(const control::Pose &robot_pose, float field_heading_rad);
    static control::Pose _field_to_robot(const control::Pose &field_pose, float field_heading_rad);
    static PoseReferenceError _get_pose_reference_error(const control::Pose &reference,
                                                        const control::Pose &measured);
    static void _update_velocity_reference(control::Pose &reference,
                                           const ReferenceVelocity &velocity,
                                           const control::Pose &measured,
                                           float dt_s);
    static void _update_reference_pid_stop_state(const PoseReferenceError &error,
                                                 bool translation_requested,
                                                 bool heading_requested,
                                                 bool &position_pid_stopped,
                                                 bool &heading_pid_stopped);

    void _handle_mode_change(pb_size_t command_tag);
    void _begin_pose_command(const robot_DriveCommand &cmd,
                             const control::PoseSnapshot &pose_snapshot);
    void _begin_tape_alignment_command(const robot_DriveCommand &cmd,
                                       const control::PoseSnapshot &pose_snapshot);

    void _update_velocity(control::Drivetrain &drivetrain,
                          const robot_VelocityCommand &velocity,
                          const control::PoseSnapshot &pose_snapshot,
                          float dt_s);
    void _update_pose(control::Drivetrain &drivetrain,
                      const control::PoseSnapshot &pose_snapshot,
                      float dt_s);
    void _update_tape_alignment(control::Drivetrain &drivetrain,
                                const control::PoseSnapshot &pose_snapshot,
                                float dt_s);
    void _update_tape_follow(control::Drivetrain &drivetrain,
                             const robot_TapeFollowCommand &command,
                             float dt_s);
    void _drive_to_reference(control::Drivetrain &drivetrain,
                             const PoseReferenceError &error,
                             bool position_pid_stopped,
                             bool heading_pid_stopped,
                             float dt_s,
                             float minimum_translation_command = 0.0f);
    void _reset_pose_pids();

    pb_size_t _command_tag = robot_DriveCommand_stop_tag;
    uint32_t _previous_pose_sequence = 0;
    uint32_t _previous_tape_alignment_sequence = 0;

    VelocityState _velocity{};
    PoseState _pose{};
    TapeAlignmentDriveState _tape_alignment{};

    control::PID _x_pid{62.0f, 0.0f, 0.0f, 30.0f, -80.0f, 80.0f};
    control::PID _y_pid{42.0f, 0.0f, 0.0f, 30.0f, -80.0f, 80.0f};
    control::PID _heading_pid{29.0f, 0.0f, 2.3f, 40.0f, -70.0f, 70.0f};
    control::TapePID _tape_pid{5.0f, 0.0f, 0.05f};

    std::atomic_bool _reached_pose{false};
    std::atomic_uint32_t _fault{0};
};
