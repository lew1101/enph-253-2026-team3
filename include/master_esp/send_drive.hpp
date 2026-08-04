#pragma once

#include <initializer_list>

#include "control/pose_estimator.hpp"

#include "drive.pb.h"

using control::Pose;

constexpr float DEFAULT_PATH_SPEED_MPS = 0.5f;

constexpr robot_DriveCommand make_pose_drive_command(
    const Pose &pose, bool is_relative = false, float desired_speed_mps = DEFAULT_PATH_SPEED_MPS)
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

constexpr robot_DriveCommand make_tape_alignment_drive_command(float staging_direction = -1.0f)
{
    robot_DriveCommand out = robot_DriveCommand_init_zero;
    out.which_command = robot_DriveCommand_tape_alignment_tag;
    out.command.tape_alignment.staging_direction = staging_direction;
    return out;
}

bool drive_fault_active();
void send_velocity(float vx_percent, float vy_percent, float omega_percent = 0.0f);
void send_stop();
esp_err_t send_pose(const Pose &waypoint,
                    bool is_relative = false,
                    uint32_t *out_sequence = nullptr,
                    float desired_speed_mps = DEFAULT_PATH_SPEED_MPS);
esp_err_t wait_for_drive_sequence(uint32_t sequence, TickType_t timeout);
esp_err_t send_drive_command_and_wait(const robot_DriveCommand &command,
                                      TickType_t timeout = pdMS_TO_TICKS(5000));
esp_err_t send_tape_alignment_and_wait(float staging_direction = -1.0f,
                                       TickType_t timeout = pdMS_TO_TICKS(6000));
esp_err_t send_pose_and_wait(const Pose &waypoint,
                             bool relative = false,
                             TickType_t timeout = pdMS_TO_TICKS(5000),
                             float desired_speed_mps = DEFAULT_PATH_SPEED_MPS);
esp_err_t send_pose_through(const Pose &waypoint,
                            float pass_radius_m = 0.15f,
                            TickType_t timeout = pdMS_TO_TICKS(5000),
                            float desired_speed_mps = DEFAULT_PATH_SPEED_MPS);
[[noreturn]] void halt_autonomous(const char *phase, esp_err_t err);
void follow_route(std::initializer_list<Pose> route,
                  float desired_speed_mps = DEFAULT_PATH_SPEED_MPS);
