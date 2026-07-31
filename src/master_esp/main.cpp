#include "freertos/idf_additions.h"
#include <cmath>
#include <initializer_list>

#include "esp_err.h"
#include "esp_check.h"

#include "shared/robot_flags.hpp"
#include "supervisor.hpp"
#include "drive_pose.hpp"
#include "waypoints.hpp"
#include "front_chassis.hpp"

#include "tasks/uart.hpp"
#include "tasks/camera_uart.hpp"
#include "tasks/metal.hpp"

#include "actuators/limit.hpp"

using control::Pose;

static constexpr char TAG[]{"master_main"};

namespace {

static DebouncedLimitSwitch guide_limit_switch{GPIO_NUM_9};

enum ElevatorPos : int32_t {
    ELEV_FLOOR = 0,
    ELEV_TOWER_1 = 500,
    ELEV_TOWER_2_PLUS = 4000,
    ELEV_TOWER_2_MINUS = 3400,
    ELEV_TOWER_2 = 500,
    ELEV_TOWER_3_PLUS = 4000,
    ELEV_TOWER_3_MINUS = 3400,
    ELEV_TOWER_3 = 500,
    ELEV_BACK = 5000,
    ELEV_ROCK = 0,
};

enum SpearPos : int32_t { //
    SPEAR_LEFT = 50'000,
    SPEAR_CENTRE = 24'000,
    SPEAR_RIGHT = 0,
    SPEAR_CENTERING = 0
};

constexpr float SPEAR_UP_DEG = 55.0f;
constexpr float SPEAR_UP_SLIGHTLY = 135.0f;
constexpr float SPEAR_DOWN_DEG = 145.0f;
constexpr float SPEAR_MOVE_DELAY = 600.0f; // ms

constexpr float CLAW_OPEN_DEG = 90.0f;
constexpr float CLAW_CLOSE_TOWER_DEG = 35.0f;
constexpr float CLAW_CLOSE_ROCK_DEG = 70.0f;

constexpr float CLAW_TOWER_DELAY = 500.0f; // ms

void follow_route(std::initializer_list<WaypointIndex> route)
{
    size_t index = 0;
    for (const WaypointIndex waypoint : route) {
        const Pose &pose = WAYPOINTS[waypoint];
        const bool is_last = ++index == route.size();
        const esp_err_t err = is_last ? send_pose_and_wait(pose) : send_pose_through(pose);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "route failed at waypoint %u", static_cast<unsigned>(waypoint));
            halt_autonomous("pose route", err);
        }
    }
}

esp_err_t rocks_sequence()
{
    using metal_detector::MetalDetector;
    using metal_detector::MetalState;

    constexpr TickType_t METAL_CALIBRATION_WAIT_DELAY = pdMS_TO_TICKS(15000);
    constexpr TickType_t METAL_WAIT_DELAY = pdMS_TO_TICKS(3000);

    xEventGroupClearBits(supervisor::g_robot_control_flags, robot_flags::CONTROL_ACTUATORS);
    xEventGroupClearBits(supervisor::g_robot_status_flags,
                         robot_flags::STATUS_METAL_CALIBRATED_MASK |
                             robot_flags::STATUS_METAL_SEEN_MASK);
    xEventGroupSetBits(supervisor::g_robot_control_flags,
                       robot_flags::CONTROL_DRIVE_ENABLED | robot_flags::CONTROL_METAL_ENABLED);

    struct MetalControlGuard {
        ~MetalControlGuard()
        {
            xEventGroupClearBits(supervisor::g_robot_control_flags,
                                 robot_flags::CONTROL_METAL_ENABLED);
        }
    } metal_control_guard;

    // SEQUENCE START
    auto status_flags = xEventGroupWaitBits(supervisor::g_robot_status_flags,
                                            robot_flags::STATUS_METAL_CALIBRATED_MASK,
                                            pdFALSE,
                                            pdTRUE,
                                            METAL_CALIBRATION_WAIT_DELAY);

    if (!robot_flags::has_all_flags(status_flags, robot_flags::STATUS_METAL_CALIBRATED_MASK)) {
        ESP_LOGE(TAG, "metal detector calibration timed out; skipping rocks");
        return ESP_FAIL;
    }

    bool rock_was_found = false;

    const auto pick_up_and_go_to_next = [&](const Pose &waypoint) -> esp_err_t {
        elevator_claw.move_to_position(ElevatorPos::ELEV_FLOOR);
        elevator_claw.set_claw(CLAW_CLOSE_ROCK_DEG);

        delay(500);
        const esp_err_t send_err = send_pose(waypoint);
        if (send_err != ESP_OK) return send_err;

        elevator_claw.move_to_position(ElevatorPos::ELEV_BACK);
        elevator_claw.set_claw(CLAW_OPEN_DEG);

        rock_was_found = true;
        return ESP_OK;
    };

    // is_check_md_left: true for check md_1, false for check md_2
    const auto scan_if_not_found = [&](bool is_check_md_left) -> bool {
        if (rock_was_found) return false;
        auto status_flags = xEventGroupWaitBits(supervisor::g_robot_status_flags,
                                                robot_flags::STATUS_METAL_SEEN_MASK,
                                                pdFALSE,
                                                pdFALSE,
                                                METAL_WAIT_DELAY);

        if (!robot_flags::has_any_flag(status_flags, robot_flags::STATUS_METAL_SEEN_MASK)) {
            ESP_LOGI(TAG, "metal not detected.");
            return false;
        }

        if (is_check_md_left &&
            robot_flags::has_flag(status_flags, robot_flags::STATUS_METAL_1_SEEN)) {
            ESP_LOGI(TAG, "metal detected on the left");
            return true;
        } else if (robot_flags::has_flag(status_flags, robot_flags::STATUS_METAL_2_SEEN)) {
            ESP_LOGI(TAG, "metal detected on the right");
            return true;
        }

        ESP_LOGI(TAG, "metal detected on opposite side!?");
        return false;
    };

    ESP_RETURN_ON_ERROR(
        send_pose_and_wait(WAYPOINTS[POSE_ROCK_SCAN_1]), TAG, "failed to reach rock scan 1");

    if (scan_if_not_found(false)) {
        follow_route({
            POSE_ROCK_PICKUP_1_1,
            POSE_ROCK_PICKUP_1_2,
            POSE_ROCK_PICKUP_1_3,
        });

        ESP_RETURN_ON_ERROR(pick_up_and_go_to_next(WAYPOINTS[POSE_ROCK_SCAN_3]),
                            TAG,
                            "failed to leave rock pickup 1");

    } else {
        ESP_RETURN_ON_ERROR(
            send_pose_and_wait(WAYPOINTS[POSE_ROCK_SCAN_2]), TAG, "failed to reach rock scan 2");

        if (scan_if_not_found(true)) {
            follow_route({
                POSE_ROCK_PICKUP_2_1,
                POSE_ROCK_PICKUP_2_2,
                POSE_ROCK_PICKUP_2_3,
            });

            ESP_RETURN_ON_ERROR(pick_up_and_go_to_next(WAYPOINTS[POSE_ROCK_INTER_23]),
                                TAG,
                                "failed to leave rock pickup 2");
        }
    }

    follow_route({
        POSE_ROCK_INTER_23,
        POSE_ROCK_SCAN_3,
    });

    if (scan_if_not_found(false)) {
        follow_route({
            POSE_ROCK_PICKUP_3_1,
            POSE_ROCK_PICKUP_3_2,
        });

        ESP_RETURN_ON_ERROR(pick_up_and_go_to_next(WAYPOINTS[POSE_ROCK_INTER_34_1]),
                            TAG,
                            "failed to leave rock pickup 3");
    }

    follow_route({
        POSE_ROCK_INTER_34_1,
        POSE_ROCK_SCAN_4,
    });

    if (scan_if_not_found(false)) {
        follow_route({
            POSE_ROCK_PICKUP_4_1,
            POSE_ROCK_PICKUP_4_2,
        });

        ESP_RETURN_ON_ERROR(pick_up_and_go_to_next(WAYPOINTS[POSE_ROCK_INTER_45_1]),
                            TAG,
                            "failed to leave rock pickup 4");
    }

    follow_route({
        POSE_ROCK_INTER_45_1,
        POSE_ROCK_INTER_45_2,
        POSE_ROCK_SCAN_5,
    });
    if (scan_if_not_found(true)) {
        follow_route({
            POSE_ROCK_PICKUP_5_1,
            POSE_ROCK_PICKUP_5_2,
            POSE_ROCK_PICKUP_5_3,
        });

        ESP_RETURN_ON_ERROR(pick_up_and_go_to_next(WAYPOINTS[POSE_INTER_ROCK_TOWER]),
                            TAG,
                            "failed to leave rock pickup 5");
    }

    if (!rock_was_found) {
        ESP_RETURN_ON_ERROR(pick_up_and_go_to_next(WAYPOINTS[POSE_INTER_ROCK_TOWER]),
                            TAG,
                            "failed to leave fallback rock pickup");
    }

    return ESP_OK;
}

esp_err_t build_tower_sequence()
{
    // ===============
    // Build the tower
    /*
    -> Forward -> Spear up -> Backward and center worm ->
    Elevator down -> Claw close -> Elevator up -> Spear down -> Forward
    -> Spear up -> Backward and center worm -> Elevator down -> Claw close -> Elevator up ->
    Spear down
    -> Worm to third tower piece -> Forward
    -> Spear up -> Backward and center worm -> Elevator down -> Claw close -> Elevator up ->
    Spear down
    -> Worm center cresent moon while driving backward -> rotate CW until find tape -> Tape
    follow
    -> Boss tape detected -> forward and back until tape aligned with side sensors -> rotate CCW
    until side tape aligned with front sensors -> Drive forward slowly (cresent moon will auto
    align the robot with the boss) -> Elevator down -> Claw open */
    constexpr float Y_OFFSET = 0.2f; // m

    robot_DriveCommand forward = make_pose_drive_command({0.0f, Y_OFFSET, 0.0f}, true);
    robot_DriveCommand backward = make_pose_drive_command({0.0f, -Y_OFFSET, 0.0f}, true);

    xEventGroupClearBits(supervisor::g_robot_control_flags, robot_flags::CONTROL_ACTUATORS);
    xEventGroupSetBits(supervisor::g_robot_control_flags, robot_flags::CONTROL_DRIVE_ENABLED);

    // TODO: tower building recalibration sequence
    // move to the tower
    follow_route({
        POSE_INTER_ROCK_TOWER,
        POSE_TOWER_BUILD,
    });

    // PIECE 1
    // Starting with worm spear down, claw open, elevator up, and robot facing tower
    worm_spear.set_spear(SPEAR_DOWN_DEG);
    delay(1000);

    elevator_claw.move_to_position(ElevatorPos::ELEV_TOWER_1);
    elevator_claw.set_claw(CLAW_OPEN_DEG);
    worm_spear.move_to_position(SpearPos::SPEAR_LEFT);
    ESP_RETURN_ON_ERROR(
        send_drive_command_and_wait(forward), TAG, "failed tower piece 1 forward move");

    worm_spear.set_spear(SPEAR_UP_SLIGHTLY);
    worm_spear.move_to_position(SpearPos::SPEAR_CENTRE);
    uint32_t movement_sequence = 0;
    ESP_RETURN_ON_ERROR(send_drive_command(backward, portMAX_DELAY, &movement_sequence),
                        TAG,
                        "failed to start tower piece 1 backward move");

    worm_spear.set_spear(SPEAR_UP_DEG);
    elevator_claw.move_to_position(ElevatorPos::ELEV_TOWER_1);

    elevator_claw.set_claw(CLAW_CLOSE_TOWER_DEG);
    delay(CLAW_TOWER_DELAY);
    elevator_claw.move_to_position(ElevatorPos::ELEV_TOWER_2_PLUS);
    ESP_LOGI(TAG, "Tower assembly sequence: Piece 1 complete.");

    worm_spear.set_spear(SPEAR_DOWN_DEG);
    // delay(1500); // TEMPORARY DELAY, REMOVE LATER
    ESP_RETURN_ON_ERROR(wait_for_drive_sequence(movement_sequence, pdMS_TO_TICKS(5000)),
                        TAG,
                        "tower piece 1 backward move timed out");

    // PIECE 2
    ESP_RETURN_ON_ERROR(
        send_drive_command_and_wait(forward), TAG, "failed tower piece 2 forward move");

    worm_spear.set_spear(SPEAR_UP_DEG);
    delay(500);
    elevator_claw.move_to_position(ElevatorPos::ELEV_TOWER_2_MINUS);

    elevator_claw.set_claw(CLAW_OPEN_DEG);
    delay(CLAW_TOWER_DELAY);
    elevator_claw.move_to_position(ElevatorPos::ELEV_TOWER_2);

    elevator_claw.set_claw(CLAW_CLOSE_TOWER_DEG);
    delay(CLAW_TOWER_DELAY);
    elevator_claw.move_to_position(ElevatorPos::ELEV_TOWER_3_PLUS);

    ESP_RETURN_ON_ERROR(send_drive_command(backward, portMAX_DELAY, &movement_sequence),
                        TAG,
                        "failed to start tower piece 2 backward move");

    ESP_LOGI(TAG, "Tower assembly sequence: Piece 2 complete.");
    worm_spear.set_spear(SPEAR_DOWN_DEG);
    worm_spear.move_to_position(SpearPos::SPEAR_RIGHT);

    ESP_RETURN_ON_ERROR(wait_for_drive_sequence(movement_sequence, pdMS_TO_TICKS(5000)),
                        TAG,
                        "tower piece 2 backward move timed out");

    // PIECE 3
    ESP_RETURN_ON_ERROR(
        send_drive_command_and_wait(forward), TAG, "failed tower piece 3 forward move");

    worm_spear.set_spear(SPEAR_UP_SLIGHTLY);
    worm_spear.move_to_position(SpearPos::SPEAR_CENTRE);

    worm_spear.set_spear(SPEAR_UP_DEG);
    delay(500);
    elevator_claw.move_to_position(ElevatorPos::ELEV_TOWER_3_MINUS);

    elevator_claw.set_claw(CLAW_OPEN_DEG);
    delay(CLAW_TOWER_DELAY);
    elevator_claw.move_to_position(ElevatorPos::ELEV_TOWER_3);

    elevator_claw.set_claw(CLAW_CLOSE_TOWER_DEG);
    delay(CLAW_TOWER_DELAY);
    elevator_claw.move_to_position(ElevatorPos::ELEV_TOWER_3_PLUS);

    ESP_LOGI(TAG, "Tower assembly sequence: Piece 3 complete.");
    worm_spear.set_spear(SPEAR_DOWN_DEG);
    worm_spear.move_to_position(SpearPos::SPEAR_CENTERING);

    ESP_RETURN_ON_ERROR(
        send_drive_command_and_wait(backward), TAG, "failed tower piece 3 backward move");

    return ESP_OK;
}

esp_err_t stack_tower_sequence()
{
    // TODO tower nub centring sequence...

    ESP_RETURN_ON_ERROR(
        send_pose_and_wait(WAYPOINTS[POSE_TOWER_STACK]), TAG, "failed to reach tower stack pose");

    guide_limit_switch.register_pressed_callback(
        [](void *arg) {
            auto *_elevator_claw = reinterpret_cast<control::ElevatorClaw *>(arg);

            _elevator_claw->move_to_position(ELEV_FLOOR);
            _elevator_claw->set_claw(CLAW_OPEN_DEG);
        },
        &elevator_claw);

    return ESP_OK;
}

esp_err_t solar()
{
    ESP_RETURN_ON_ERROR(
        send_pose_and_wait(WAYPOINTS[POSE_SOLAR_ALIGN]), TAG, "failed to reach solar alignment");
    delay(1000);
    ESP_RETURN_ON_ERROR(
        send_pose_and_wait(WAYPOINTS[POSE_SOLAR_PULL]), TAG, "failed to reach solar pull pose");

    return ESP_OK;
}

esp_err_t celebration_sequence() { return ESP_OK; }

} // namespace

void setup()
{
    Serial.begin(115200);
    delay(1000);

    supervisor::init();
    supervisor::attach_main_loop();

    ESP_ERROR_CHECK(start_master_uart_tasks());
    ESP_ERROR_CHECK(front_chassis_init());
    ESP_ERROR_CHECK(start_metal_detector_task());

    const esp_err_t camera_uart_err = start_camera_uart_task();
    log_i("camera UART task start: %s", esp_err_to_name(camera_uart_err));
    delay(1000);

    elevator_claw.calibrate();
    worm_spear.calibrate();
} // namespace

void loop()
{
    // INITILIZATION:

    // elevator_claw.move_to_position(ELEV_TOWER_2);
    // worm_spear.move_to_position(SPEAR_LEFT);
    // worm_spear.set_spear(SPEAR_UP_DEG, SPEAR_MOVE_DELAY);

    // PHASE 1: ROCKS and TELETUBBY

    esp_err_t err = rocks_sequence();
    if (err != ESP_OK) halt_autonomous("rocks sequence", err);

    // PHASE 2: TOWER
    //===============
    // Reorient to the tower:
    /* Tower tape detected -> forward and back until tape aligned with side sensors -> rotate CCW
    until side tape aligned with front sensors */

    err = build_tower_sequence();
    if (err != ESP_OK) halt_autonomous("tower build sequence", err);

    err = stack_tower_sequence();
    if (err != ESP_OK) halt_autonomous("tower stack sequence", err);

    // PHASE 3: SOLAR PANEL
    err = solar();
    if (err != ESP_OK) halt_autonomous("solar sequence", err);

    celebration_sequence(); // yahoo

    vTaskDelay(portMAX_DELAY);
    __builtin_unreachable();
}
