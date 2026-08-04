#include "freertos/idf_additions.h"
#include "portmacro.h"

#include <atomic>
#include <cmath>

#include "esp_err.h"
#include "esp_check.h"

#include "projdefs.h"
#include "shared/robot_flags.hpp"
#include "supervisor.hpp"

#include "send_drive.hpp"
#include "waypoints.hpp"
#include "front_chassis.hpp"

#include "tasks/camera_uart.hpp"
#include "tasks/uart.hpp"
#include "tasks/metal.hpp"

#include "sensors/metal_detector.hpp"
#include "drivers/rmt_tx.hpp"

#define RUN_WAYPOINT_TEST_ENABLED 1
#define RUN_WAYPOINT_PICKUP_ENABLED 1
#define RUN_WAYPOINT_ALIGN_ENABLED 1

using control::Pose;
using driver::DebouncedLimitSwitch;

static constexpr char TAG[]{"master_main"};

esp_err_t setup_autonomous();
esp_err_t start_autonomous_task();

namespace {

constexpr gpio_num_t GUIDE_LIM_SWITCH_PIN = GPIO_NUM_9; // debounced
constexpr gpio_num_t ENABLE_SWITCH_PIN = GPIO_NUM_14;   // debounced
constexpr gpio_num_t TRACK_SWITCH_PIN = GPIO_NUM_40;    // no debounce

constexpr gpio_num_t STATUS_LED_PIN = GPIO_NUM_13;

driver::RmtTx s_status_led_rmt_tx{{
    .gpio = STATUS_LED_PIN,
    .resolution_hz = 10'000, // 1 tick = 100 µs
    .memory_symbols = 48,
    .queue_depth = 1,
}};

DebouncedLimitSwitch s_guide_limit_switch{GUIDE_LIM_SWITCH_PIN, INPUT_PULLUP};
DebouncedLimitSwitch s_enable_limit_switch{ENABLE_SWITCH_PIN, INPUT_PULLUP};

TaskHandle_t g_autonomous_task = nullptr;

std::atomic_bool setup_completed{false};

bool worm_calibrated = false;
bool elev_calibrated = false;

bool metal_detector_available = false;
bool camera_available = false;
bool guide_switch_available = false;
bool front_chassis_available = false;

bool is_track_a;

inline void status_double_flash()
{
    static constexpr rmt_symbol_word_t DOUBLE_BLINK_MSG[2]{
        {
            .duration0 = 2000, // ON 200 ms
            .level0 = 1,
            .duration1 = 1800, // OFF 180 ms
            .level1 = 0,
        },
        {
            .duration0 = 2000, // ON 200 ms
            .level0 = 1,
            .duration1 = 6200, // OFF 620 ms
            .level1 = 0,
        },
    };

    ESP_ERROR_CHECK_WITHOUT_ABORT(s_status_led_rmt_tx.transmit(DOUBLE_BLINK_MSG));
}

#if RUN_WAYPOINT_TEST_ENABLED
enum ElevatorPos : int32_t {
    ELEV_FLOOR = 0,
    ELEV_TOWER_1 = 1000,
    ELEV_TOWER_2_PLUS = 4000,
    ELEV_TOWER_2_MINUS = 3400,
    ELEV_TOWER_2 = 1000,
    ELEV_TOWER_3_PLUS = 4000,
    ELEV_TOWER_3_MINUS = 3400,
    ELEV_TOWER_3 = 1000,
    ELEV_TOP_FRONT = 4700,
    ELEV_TIPPITY_TOP = 5000,
    ELEV_BACK = 5400,
};

enum SpearPos : int32_t { //
    SPEAR_LEFT = 55'500,
    SPEAR_CENTRE = 27'750,
    SPEAR_RIGHT = 0,
    SPEAR_CENTERING = 0
};

//Spear Angle
constexpr float SPEAR_UP_DEG = 80.0f;
constexpr float SPEAR_UP_TILTED = 75.0f;
constexpr float SPEAR_UP_SLIGHTLY = 10.0f;
constexpr float SPEAR_DOWN_DEG = 0.0f;
constexpr float SPEAR_DOWN_TILTED = 5.0f;
constexpr float MOON_OUT_OF_WAY_DEG = 100.0f;
constexpr float SPEAR_MOVE_DELAY = 600UL; // ms

constexpr float CLAW_OPEN_DEG = 90.0f;
constexpr float CLAW_CLOSE_TOWER_DEG = 15.0f;
constexpr float CLAW_CLOSE_ROCK_DEG = 70.0f;

constexpr float CLAW_TOWER_DELAY = 500.0f; // ms

void test_course()
{
    xEventGroupSetBits(supervisor::g_robot_control_flags,
                       robot_flags::CONTROL_DRIVE_ENABLED | robot_flags::CONTROL_TAPE_ENABLED);

    const WaypointIndex POSE_TOWER_BUILD = is_track_a ? POSE_TOWER_BUILD_A : POSE_TOWER_BUILD_B;
    const WaypointIndex POSE_TOWER_STACK = is_track_a ? POSE_TOWER_STACK_A : POSE_TOWER_STACK_B;

#if RUN_WAYPOINT_PICKUP_ENABLED
    send_pose_and_wait(WAYPOINTS[POSE_ROCK_SCAN_1]);
    delay(1500);
    follow_route({
        WAYPOINTS[POSE_ROCK_PICKUP_1_1],
        WAYPOINTS[POSE_ROCK_PICKUP_1_2],
        WAYPOINTS[POSE_ROCK_PICKUP_1_3],
    });
    delay(1500);
    send_pose_and_wait(WAYPOINTS[POSE_ROCK_SCAN_2]);
    delay(1500);
    follow_route({
        WAYPOINTS[POSE_ROCK_PICKUP_2_1],
        WAYPOINTS[POSE_ROCK_PICKUP_2_2],
        WAYPOINTS[POSE_ROCK_PICKUP_2_3],
    });
    delay(1500);
    follow_route({
        WAYPOINTS[POSE_ROCK_INTER_23],
        WAYPOINTS[POSE_ROCK_SCAN_3],
    });
    delay(1500);
    follow_route({
        WAYPOINTS[POSE_ROCK_PICKUP_3_1],
        WAYPOINTS[POSE_ROCK_PICKUP_3_2],
    });
    delay(1500);
    follow_route({
        WAYPOINTS[POSE_ROCK_INTER_34_1],
        WAYPOINTS[POSE_ROCK_SCAN_4],
    });
    delay(1500);
    follow_route({
        WAYPOINTS[POSE_ROCK_PICKUP_4_1],
        WAYPOINTS[POSE_ROCK_PICKUP_4_2],
        WAYPOINTS[POSE_ROCK_PICKUP_4_3],
    });
    delay(1500);
    follow_route({
        WAYPOINTS[POSE_ROCK_INTER_45_1],
        WAYPOINTS[POSE_ROCK_INTER_45_2],
        WAYPOINTS[POSE_ROCK_SCAN_5],
    });
    delay(1500);
    follow_route({
        WAYPOINTS[POSE_ROCK_PICKUP_5_1],
        WAYPOINTS[POSE_ROCK_PICKUP_5_2],
        WAYPOINTS[POSE_ROCK_PICKUP_5_3],
    });
    delay(1500);
    send_pose_and_wait(WAYPOINTS[POSE_ROCK_SCAN_6]);
    delay(1500);
    follow_route({
        WAYPOINTS[POSE_ROCK_PICKUP_6_1],
        WAYPOINTS[POSE_ROCK_PICKUP_6_2],
        WAYPOINTS[POSE_ROCK_PICKUP_6_3],
    });
    delay(1500);
    follow_route({
        WAYPOINTS[POSE_INTER_ROCK_TOWER],
        WAYPOINTS[POSE_TOWER_BUILD],
    });
    delay(5000);

#if RUN_WAYPOINT_ALIGN_ENABLED
    send_tape_alignment_and_wait(-1.0f);
    delay(5000);
#endif

    send_pose_and_wait(WAYPOINTS[POSE_TOWER_STACK]);
    delay(5000);

#if RUN_WAYPOINT_ALIGN_ENABLED
    send_tape_alignment_and_wait(1.0f);
    delay(5000);
#endif

    send_pose_and_wait(WAYPOINTS[POSE_SOLAR_ALIGN]);
    delay(500);
    follow_route({
        WAYPOINTS[POSE_SOLAR_PULL],
        WAYPOINTS[POSE_SOLAR_TURN],
    });
#else
    send_pose_and_wait(WAYPOINTS[POSE_ROCK_SCAN_1]);
    send_pose_and_wait(WAYPOINTS[POSE_ROCK_SCAN_2]);
    follow_route({
        WAYPOINTS[POSE_ROCK_INTER_23],
        WAYPOINTS[POSE_ROCK_SCAN_3],
        WAYPOINTS[POSE_ROCK_INTER_34_1],
        WAYPOINTS[POSE_ROCK_SCAN_4],
    });
    follow_route({
        WAYPOINTS[POSE_ROCK_INTER_45_1],
        WAYPOINTS[POSE_ROCK_INTER_45_2],
        WAYPOINTS[POSE_ROCK_SCAN_5],
    });
    send_pose_and_wait(WAYPOINTS[POSE_ROCK_SCAN_6]);
    follow_route({
        WAYPOINTS[POSE_INTER_ROCK_TOWER],
        WAYPOINTS[POSE_TOWER_BUILD],
    });
    delay(5000);

#if RUN_WAYPOINT_ALIGN_ENABLED
    send_tape_alignment_and_wait(-1.0f);
    delay(5000);
#endif

    send_pose_and_wait(WAYPOINTS[POSE_TOWER_STACK]);
    delay(5000);

#if RUN_WAYPOINT_ALIGN_ENABLED
    send_tape_alignment_and_wait(1.0f);
    delay(5000);
#endif

    send_pose_and_wait(WAYPOINTS[POSE_SOLAR_ALIGN]);
    delay(500);
    follow_route({
        WAYPOINTS[POSE_SOLAR_PULL],
        WAYPOINTS[POSE_SOLAR_TURN],
    });
#endif
}
#endif

esp_err_t rocks_sequence()
{
    using metal_detector::MetalDetector;
    using metal_detector::MetalState;

    constexpr TickType_t METAL_CALIBRATION_TIMEOUT = pdMS_TO_TICKS(15000);
    constexpr TickType_t MD_TIMEOUT = pdMS_TO_TICKS(3000);

    xEventGroupClearBits(supervisor::g_robot_control_flags, robot_flags::CONTROL_ACTUATORS);
    xEventGroupClearBits(supervisor::g_robot_status_flags,
                         robot_flags::STATUS_METAL_CALIBRATED_MASK |
                             robot_flags::STATUS_METAL_SEEN_MASK);
    xEventGroupSetBits(supervisor::g_robot_control_flags,
                       robot_flags::CONTROL_DRIVE_ENABLED | robot_flags::CONTROL_METAL_ENABLED);

    // ensure control metal enabled bits are cleared when function out of scope
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
                                            METAL_CALIBRATION_TIMEOUT);

    if (!robot_flags::has_all_flags(status_flags, robot_flags::STATUS_METAL_CALIBRATED_MASK)) {
        ESP_LOGE(TAG, "metal detector calibration timed out; skipping rocks");
        return ESP_ERR_NOT_FOUND;
    }

    bool rock_was_found = false;

    const auto pick_up_and_go_to_next = [&](const Pose &waypoint) -> esp_err_t {
        rock_was_found = true;

        elevator_claw.move_to_position(ElevatorPos::ELEV_FLOOR);
        elevator_claw.set_claw(CLAW_CLOSE_ROCK_DEG);

        delay(500);
        ESP_RETURN_ON_ERROR(send_pose(waypoint), TAG, "failed to send pose to next waypoint");

        elevator_claw.move_to_position(ElevatorPos::ELEV_TOP_FRONT);
        elevator_claw.move_to_position(ElevatorPos::ELEV_TIPPITY_TOP);
        elevator_claw.move_to_position(ElevatorPos::ELEV_BACK);
        elevator_claw.set_claw(CLAW_OPEN_DEG);
        elevator_claw.move_to_position(ElevatorPos::ELEV_TIPPITY_TOP);
        elevator_claw.move_to_position(ElevatorPos::ELEV_TOP_FRONT);

        return ESP_OK;
    };

    // is_check_md_left: true for check md_1, false for check md_2
    const auto scan_if_not_found = [&](bool is_check_md_left) -> bool {
        if (rock_was_found) return false;
        auto status_flags = xEventGroupWaitBits(supervisor::g_robot_status_flags,
                                                robot_flags::STATUS_METAL_SEEN_MASK,
                                                pdFALSE,
                                                pdFALSE,
                                                MD_TIMEOUT);

        bool metal_detected =
            (is_check_md_left &&
             robot_flags::has_flag(status_flags, robot_flags::STATUS_METAL_1_SEEN)) ||
            (!is_check_md_left &&
             robot_flags::has_flag(status_flags, robot_flags::STATUS_METAL_2_SEEN));

        if (metal_detected) {
            ESP_LOGI(
                TAG, "metal detected on the %s", is_check_md_left ? "left side" : "right side");
            status_double_flash();
            return true;
        }

        ESP_LOGI(TAG, "metal not detected");
        return false;
    };

    ESP_RETURN_ON_ERROR(
        send_pose_and_wait(WAYPOINTS[POSE_ROCK_SCAN_1]), TAG, "failed to reach rock scan 1");

    if (scan_if_not_found(false)) {
        follow_route({
            WAYPOINTS[POSE_ROCK_PICKUP_1_1],
            WAYPOINTS[POSE_ROCK_PICKUP_1_2],
            WAYPOINTS[POSE_ROCK_PICKUP_1_3],
        });

        ESP_RETURN_ON_ERROR(pick_up_and_go_to_next(WAYPOINTS[POSE_ROCK_SCAN_3]),
                            TAG,
                            "failed to leave rock pickup 1");

    } else {
        ESP_RETURN_ON_ERROR(
            send_pose_and_wait(WAYPOINTS[POSE_ROCK_SCAN_2]), TAG, "failed to reach rock scan 2");

        if (scan_if_not_found(true)) {
            follow_route({
                WAYPOINTS[POSE_ROCK_PICKUP_2_1],
                WAYPOINTS[POSE_ROCK_PICKUP_2_2],
                WAYPOINTS[POSE_ROCK_PICKUP_2_3],
            });

            ESP_RETURN_ON_ERROR(pick_up_and_go_to_next(WAYPOINTS[POSE_ROCK_INTER_23]),
                                TAG,
                                "failed to leave rock pickup 2");
        }
    }

    follow_route({
        WAYPOINTS[POSE_ROCK_INTER_23],
        WAYPOINTS[POSE_ROCK_SCAN_3],
    });

    if (scan_if_not_found(false)) {
        follow_route({
            WAYPOINTS[POSE_ROCK_PICKUP_3_1],
            WAYPOINTS[POSE_ROCK_PICKUP_3_2],
        });

        ESP_RETURN_ON_ERROR(pick_up_and_go_to_next(WAYPOINTS[POSE_ROCK_INTER_34_1]),
                            TAG,
                            "failed to leave rock pickup 3");
    }

    follow_route({
        WAYPOINTS[POSE_ROCK_INTER_34_1],
        WAYPOINTS[POSE_ROCK_SCAN_4],
    });

    if (scan_if_not_found(false)) {
        follow_route({
            WAYPOINTS[POSE_ROCK_PICKUP_4_1],
            WAYPOINTS[POSE_ROCK_PICKUP_4_2],
            WAYPOINTS[POSE_ROCK_PICKUP_4_3],
        });

        ESP_RETURN_ON_ERROR(pick_up_and_go_to_next(WAYPOINTS[POSE_ROCK_INTER_45_1]),
                            TAG,
                            "failed to leave rock pickup 4");
    }

    follow_route({
        WAYPOINTS[POSE_ROCK_INTER_45_1],
        WAYPOINTS[POSE_ROCK_INTER_45_2],
        WAYPOINTS[POSE_ROCK_SCAN_5],
    });

    if (scan_if_not_found(true)) {
        follow_route({
            WAYPOINTS[POSE_ROCK_PICKUP_5_1],
            WAYPOINTS[POSE_ROCK_PICKUP_5_2],
            WAYPOINTS[POSE_ROCK_PICKUP_5_3],
        });

        ESP_RETURN_ON_ERROR(pick_up_and_go_to_next(WAYPOINTS[POSE_INTER_ROCK_TOWER]),
                            TAG,
                            "failed to leave rock pickup 5");
    }

    if (!rock_was_found) {
        follow_route({
            WAYPOINTS[POSE_ROCK_PICKUP_6_1],
            WAYPOINTS[POSE_ROCK_PICKUP_6_2],
            WAYPOINTS[POSE_ROCK_PICKUP_6_3],
        });

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
    constexpr float Y_OFFSET = 0.1f; // m
    const WaypointIndex POSE_TOWER_BUILD = is_track_a ? POSE_TOWER_BUILD_A : POSE_TOWER_BUILD_B;

    const TickType_t DRIVE_TIMEOUT = pdMS_TO_TICKS(3000);

    robot_DriveCommand forward = make_pose_drive_command({0.0f, Y_OFFSET, 0.0f}, true);
    robot_DriveCommand backward = make_pose_drive_command({0.0f, -Y_OFFSET, 0.0f}, true);

    xEventGroupClearBits(supervisor::g_robot_control_flags, robot_flags::CONTROL_ACTUATORS);
    xEventGroupSetBits(supervisor::g_robot_control_flags,
                       robot_flags::CONTROL_DRIVE_ENABLED | robot_flags::CONTROL_TAPE_ENABLED);

    // move to the tower
    // follow_route({
    //     WAYPOINTS[POSE_INTER_ROCK_TOWER],
    //     WAYPOINTS[POSE_TOWER_BUILD],
    // });

    // realign with tower tape
    // stage robot to the left, so scan towards the right.
    ESP_RETURN_ON_ERROR(
        send_tape_alignment_and_wait(-1.0f), TAG, "failed to align with tower tape");

    // PIECE 1
    // Starting with worm spear down, spear on the left, claw open, elevator at TOP_FRONT, and robot facing tower
    worm_spear.set_spear(SPEAR_DOWN_DEG);
    ESP_RETURN_ON_ERROR(
        send_drive_command_and_wait(forward, DRIVE_TIMEOUT), TAG, "failed tower piece 1 forward move");

    worm_spear.set_spear(SPEAR_UP_SLIGHTLY);
    uint32_t movement_sequence = 0;
    ESP_RETURN_ON_ERROR(send_drive_command(backward, DRIVE_TIMEOUT, &movement_sequence),
                        TAG,
                        "failed to start tower piece 1 backward move");
    worm_spear.move_to_position(SpearPos::SPEAR_CENTRE);
    elevator_claw.set_claw(CLAW_OPEN_DEG);
    elevator_claw.move_to_position(ElevatorPos::ELEV_TOWER_1);

    // Pick up tower piece tilted, stack them straight
    worm_spear.set_spear(SPEAR_UP_TILTED);
    elevator_claw.set_claw(CLAW_CLOSE_TOWER_DEG);
    delay(CLAW_TOWER_DELAY);
    elevator_claw.move_to_position(ElevatorPos::ELEV_TOWER_2_PLUS);
    ESP_LOGI(TAG, "Tower assembly sequence: Piece 1 complete.");

    worm_spear.set_spear(SPEAR_DOWN_DEG);
    ESP_RETURN_ON_ERROR(wait_for_drive_sequence(movement_sequence, DRIVE_TIMEOUT),
                        TAG,
                        "tower piece 1 backward move timed out");

    // PIECE 2
    ESP_RETURN_ON_ERROR(
        send_drive_command_and_wait(forward, DRIVE_TIMEOUT), TAG, "failed tower piece 2 forward move");
    worm_spear.set_spear(SPEAR_UP_DEG);

    ESP_RETURN_ON_ERROR(send_drive_command(backward, DRIVE_TIMEOUT, &movement_sequence),
                        TAG,
                        "failed to start tower piece 2 backward move");
    elevator_claw.move_to_position(ElevatorPos::ELEV_TOWER_2_MINUS);
    elevator_claw.set_claw(CLAW_OPEN_DEG);
    delay(CLAW_TOWER_DELAY);

    worm_spear.set_spear(SPEAR_UP_TILTED);
    elevator_claw.move_to_position(ElevatorPos::ELEV_TOWER_2);
    elevator_claw.set_claw(CLAW_CLOSE_TOWER_DEG);
    delay(CLAW_TOWER_DELAY);
    elevator_claw.move_to_position(ElevatorPos::ELEV_TOWER_3_PLUS);

    ESP_LOGI(TAG, "Tower assembly sequence: Piece 2 complete.");
    worm_spear.set_spear(SPEAR_DOWN_DEG);
    worm_spear.move_to_position(SpearPos::SPEAR_RIGHT);

    ESP_RETURN_ON_ERROR(wait_for_drive_sequence(movement_sequence, DRIVE_TIMEOUT),
                        TAG,
                        "tower piece 2 backward move timed out");

    // PIECE 3
    ESP_RETURN_ON_ERROR(
        send_drive_command_and_wait(forward, DRIVE_TIMEOUT), TAG, "failed tower piece 3 forward move");

    worm_spear.set_spear(SPEAR_UP_SLIGHTLY);
    worm_spear.move_to_position(SpearPos::SPEAR_CENTRE);

    worm_spear.set_spear(SPEAR_UP_DEG);
    elevator_claw.move_to_position(ElevatorPos::ELEV_TOWER_3_MINUS);

    elevator_claw.set_claw(CLAW_OPEN_DEG);
    delay(CLAW_TOWER_DELAY);
    elevator_claw.move_to_position(ElevatorPos::ELEV_TOWER_3);

    elevator_claw.set_claw(CLAW_CLOSE_TOWER_DEG);
    delay(CLAW_TOWER_DELAY);
    elevator_claw.move_to_position(ElevatorPos::ELEV_TOWER_3_PLUS);

    ESP_LOGI(TAG, "Tower assembly sequence: Piece 3 complete.");
    ESP_RETURN_ON_ERROR(
        send_drive_command_and_wait(backward), TAG, "failed tower piece 3 backward move");

    return ESP_OK;
}

esp_err_t stack_tower_sequence()
{
    constexpr TickType_t GUIDE_TIMEOUT = pdMS_TO_TICKS(500);
    constexpr float MOVEMENT_LOOKAHEAD_M = 0.14;

    const WaypointIndex POSE_TOWER_STACK = is_track_a ? POSE_TOWER_STACK_A : POSE_TOWER_STACK_B;

    xEventGroupSetBits(supervisor::g_robot_control_flags, robot_flags::CONTROL_DRIVE_ENABLED);
    ESP_RETURN_ON_ERROR(
        send_pose_and_wait(WAYPOINTS[POSE_TOWER_STACK], false, GUIDE_TIMEOUT, MOVEMENT_LOOKAHEAD_M),
        TAG,
        "failed to reach tower stack pose");

    // realign with tower tape
    // stage toward the right, so scan towards the left.
    ESP_RETURN_ON_ERROR(send_tape_alignment_and_wait(1.0f), TAG, "failed to align with tower tape");

    if (s_guide_limit_switch.wait_until_pressed(GUIDE_TIMEOUT)) {
        send_stop(); // immediately stop as soon as we hit the guide switch

        worm_spear.set_spear(SPEAR_UP_DEG);
        elevator_claw.move_to_position(ELEV_FLOOR);
        elevator_claw.set_claw(CLAW_OPEN_DEG);
        ESP_LOGI(TAG, "guide limit switch hit and stopping on nub");
    } else {
        ESP_LOGE(TAG, "guide switch did not engage??");
        return ESP_FAIL;
    }

    return ESP_OK;
}

esp_err_t solar()
{
    xEventGroupSetBits(supervisor::g_robot_control_flags, robot_flags::CONTROL_DRIVE_ENABLED);

    ESP_RETURN_ON_ERROR(
        send_pose_and_wait(WAYPOINTS[POSE_SOLAR_ALIGN]), TAG, "failed to reach solar alignment");
    delay(500);
    follow_route({
        WAYPOINTS[POSE_SOLAR_PULL],
        WAYPOINTS[POSE_SOLAR_TURN],
    });

    return ESP_OK;
}

esp_err_t celebration_sequence() { return ESP_OK; }

void autonomous_task(void *arg)
{
    (void)arg;
    xEventGroupSetBits(supervisor::g_robot_control_flags, robot_flags::CONTROL_DRIVE_ENABLED);

    // test_course();
    // INITILIZATION:

    if (elev_calibrated) elevator_claw.move_to_position(ELEV_TOP_FRONT);
    if (worm_calibrated) {
        worm_spear.set_spear(SPEAR_DOWN_DEG, 100UL);
        worm_spear.move_to_position(SPEAR_LEFT);
        worm_spear.set_spear(SPEAR_UP_DEG, 100UL);
    }

    // // PHASE 1: ROCKS and TELETUBBY

    esp_err_t err = ESP_OK;
    // if (metal_detector_available && elev_calibrated) {
    //     err = rocks_sequence();
    //     if (err != ESP_OK && err != ESP_ERR_NOT_FOUND) halt_autonomous("rocks sequence", err);
    // } else {
    //     ESP_LOGW(TAG,
    //              "skipping rocks: metal=%s elevator=%s",
    //              metal_detector_available ? "ready" : "unavailable",
    //              elev_calibrated ? "ready" : "uncalibrated");
    // }

    // PHASE 2: TOWER
    //===============
    // Reorient to the tower:
    /* Tower tape detected -> forward and back until tape aligned with side sensors -> rotate CCW
    until side tape aligned with front sensors */

    err = build_tower_sequence();
    if (err != ESP_OK) halt_autonomous("tower build sequence", err);

    // err = stack_tower_sequence();
    // if (err != ESP_OK) halt_autonomous("tower stack sequence", err);

    // if (worm_calibrated && elev_calibrated && guide_switch_available) {
    //     err = build_tower_sequence();
    //     if (err != ESP_OK) halt_autonomous("tower build sequence", err);

    //     err = stack_tower_sequence();
    //     if (err != ESP_OK) halt_autonomous("tower stack sequence", err);
    // } else {
    //     ESP_LOGW(TAG,
    //              "skipping tower: worm=%s elevator=%s guide=%s",
    //              worm_calibrated ? "ready" : "uncalibrated",
    //              elev_calibrated ? "ready" : "uncalibrated",
    //              guide_switch_available ? "ready" : "unavailable");
    // }

    // PHASE 3: SOLAR PANEL
    // err = solar();
    // if (err != ESP_OK) halt_autonomous("solar sequence", err);

    celebration_sequence(); // yahoo

    vTaskDelay(portMAX_DELAY);
    __builtin_unreachable();
}

esp_err_t reset_autonomous_task()
{
    if (g_autonomous_task == nullptr) return start_autonomous_task();

    TaskHandle_t task_to_delete = g_autonomous_task;
    vTaskSuspend(task_to_delete);
    g_autonomous_task = nullptr;

    xEventGroupClearBits(supervisor::g_robot_control_flags, robot_flags::CONTROL_ACTUATORS);

    robot_DriveCommand stop = robot_DriveCommand_init_zero;
    stop.which_command = robot_DriveCommand_stop_tag;
    stop.command.stop.brake = true;
    const esp_err_t stop_err = send_drive_command(stop, pdMS_TO_TICKS(100));

    if (front_chassis_available) {
        elevator_claw.stop_elevator();
        worm_spear.stop_worm();
    }
    vTaskDelete(task_to_delete);

    if (stop_err != ESP_OK) {
        ESP_LOGE(TAG,
                 "failed to stop drivetrain before autonomous reset: %s",
                 esp_err_to_name(stop_err));
        return stop_err;
    }

    return start_autonomous_task();
}

} // namespace

esp_err_t setup_autonomous()
{
    if (setup_completed.load(std::memory_order_acquire)) {
        ESP_LOGI(TAG, "setup_autonomous already called once, skipping.");
        return ESP_OK;
    }

    ESP_RETURN_ON_FALSE(GUIDE_LIM_SWITCH_PIN != WORM_LIMIT_SWITCH_PIN,
                        ESP_ERR_INVALID_STATE,
                        TAG,
                        "guide and worm calibration switches cannot share GPIO %u",
                        static_cast<unsigned>(GUIDE_LIM_SWITCH_PIN));

    pinMode(TRACK_SWITCH_PIN, INPUT_PULLDOWN);
    is_track_a = digitalRead(TRACK_SWITCH_PIN) == HIGH;

    is_track_a = true; // TEMPORARY OVERRIDE FOR TESTING

    // start drivetrain UART tasks
    ESP_RETURN_ON_ERROR(
        start_master_uart_tasks(), TAG, "drivetrain UART task start failed, aborting setup.");

    // init elevator_claw and worm_spear
    const esp_err_t front_chassis_err = front_chassis_init();
    front_chassis_available = front_chassis_err == ESP_OK;
    if (!front_chassis_available)
        ESP_LOGW(TAG,
                 "front chassis unavailable; pickup and tower phases will be skipped: %s",
                 esp_err_to_name(front_chassis_err));

    // init metal detector task
    const esp_err_t metal_detector_err = start_metal_detector_task();
    metal_detector_available = metal_detector_err == ESP_OK;
    if (!metal_detector_available)
        ESP_LOGW(TAG,
                 "metal detector unavailable; rock phase will be skipped: %s",
                 esp_err_to_name(metal_detector_err));

    // init camera UART task
    const esp_err_t camera_uart_err = start_camera_uart_task();
    camera_available = camera_uart_err == ESP_OK;
    if (!camera_available)
        ESP_LOGW(TAG,
                 "camera UART unavailable; camera-dependent steps will be skipped: %s",
                 esp_err_to_name(camera_uart_err));

    delay(1000);

    constexpr bool CALIBRATE = true;

    if (CALIBRATE) {
        // calibrate elevator and worm spear if available
        if (front_chassis_available) {
            elevator_claw.set_claw(CLAW_OPEN_DEG);
            worm_spear.set_spear(SPEAR_DOWN_DEG, 100.0f);

            const esp_err_t worm_err = worm_spear.calibrate();
            worm_calibrated = worm_err == ESP_OK;
            if (!worm_calibrated)
                ESP_LOGW(TAG,
                         "worm spear calibration failed; tower phase will be skipped: %s",
                         esp_err_to_name(worm_err));

            const esp_err_t elev_err = elevator_claw.calibrate();
            elev_calibrated = elev_err == ESP_OK;
            if (!elev_calibrated)
                ESP_LOGW(TAG,
                         "elevator calibration failed; pickup and tower phases will be skipped: %s",
                         esp_err_to_name(elev_err));

            if (elev_calibrated) elevator_claw.move_to_position(ElevatorPos::ELEV_TOWER_2_MINUS);
        }
    }

    setup_completed.store(true, std::memory_order_release);

    ESP_LOGI(TAG,
             "autonomous setup complete; waiting for enable switch (metal=%s camera=%s chassis=%s "
             "guide=%s)",
             metal_detector_available ? "ready" : "skip",
             camera_available ? "ready" : "skip",
             front_chassis_available ? "ready" : "skip",
             guide_switch_available ? "ready" : "skip");
    return ESP_OK;
}

esp_err_t start_autonomous_task()
{
    constexpr uint32_t TASK_STACK_DEPTH = 8192;
    constexpr UBaseType_t TASK_PRIORITY = 4;
    constexpr BaseType_t TASK_CORE_ID = 1;

    ESP_RETURN_ON_FALSE(setup_completed.load(std::memory_order_acquire),
                        ESP_ERR_INVALID_STATE,
                        TAG,
                        "autonomous setup has not completed");
    ESP_RETURN_ON_FALSE(g_autonomous_task == nullptr,
                        ESP_ERR_INVALID_STATE,
                        TAG,
                        "autonomous task already started");

    const BaseType_t task_created = xTaskCreatePinnedToCore( //
        autonomous_task,
        "autonomous",
        TASK_STACK_DEPTH,
        nullptr,
        TASK_PRIORITY,
        &g_autonomous_task,
        TASK_CORE_ID);

    if (task_created != pdPASS) {
        g_autonomous_task = nullptr;
        return ESP_ERR_NO_MEM;
    }

    return ESP_OK;
}

void setup()
{
    Serial.begin(115200);
    delay(1000);

    s_status_led_rmt_tx.init();

    supervisor::init();
    supervisor::attach_main_loop(&g_autonomous_task);

#if !RUN_WAYPOINT_TEST_ENABLED
    if (!s_enable_limit_switch.begin("autonomous_enable_switch")) {
        ESP_LOGE(TAG, "switches: failed to start autonomous enable switch");
        return;
    }

    guide_switch_available = s_guide_limit_switch.begin("tower_guide_switch");
    if (!guide_switch_available)
        ESP_LOGW(TAG, "guide switch unavailable; tower phase will be skipped");

    s_enable_limit_switch.register_pressed_callback(
        [](void *arg) {
            (void)arg;
            if (g_autonomous_task == nullptr) {
                start_autonomous_task();
            } else {
                reset_autonomous_task();
            }
        },
        nullptr);

    const esp_err_t setup_err = setup_autonomous();
    if (setup_err != ESP_OK) {
        ESP_LOGE(TAG,
                 "autonomous setup failed: %s; mission will remain stopped",
                 esp_err_to_name(setup_err));
    }
#else
    xEventGroupSetBits(supervisor::g_robot_control_flags,
                       robot_flags::CONTROL_DRIVE_ENABLED | robot_flags::CONTROL_TAPE_ENABLED);

    // setup_autonomous();
    // start_autonomous_task();

    // test_course();
#endif

    vTaskDelete(nullptr);
}

void loop() {}

// void loop()
// {
//     static int elev_speed = 9000;
//     static int elev_accel = 1600;
//     static int worm_speed = 18000;
//     static int worm_accel = 3200;

//     if (Serial.available()) {
//         String command = Serial.readStringUntil('\n');
//         command.trim();
//         if (command.length() == 0) return;

//         // ELEVATOR COMMANDS
//         if (command.startsWith("es")) {
//             elev_speed = command.substring(2).toInt();
//             ESP_LOGI(TAG, "Setting Elevator Speed: %d Hz, Accel: %d", elev_speed, elev_accel);
//             elevator_claw.set_speed(elev_speed, elev_accel);
//         } else if (command.startsWith("ea")) {
//             elev_accel = command.substring(2).toInt();
//             ESP_LOGI(TAG, "Setting Elevator Speed: %d Hz, Accel: %d", elev_speed, elev_accel);
//             elevator_claw.set_speed(elev_speed, elev_accel);
//         } else if (command.startsWith("eh")) {
//             elevator_claw.set_home_position();
//             ESP_LOGI(TAG, "Elevator home position set.");
//         } else if (command.startsWith("ep")) {
//             int pos = elevator_claw.get_current_position();
//             ESP_LOGI(TAG, "Current Elevator Position: %d", pos);
//         } else if (command.startsWith("el")) {
//             bool limit_pressed = elevator_claw.limit_is_pressed();
//             ESP_LOGI(TAG, "Elevator limit switch pressed: %s", limit_pressed ? "true" : "false");
//         } else if (command.startsWith("ecal")) {
//             ESP_LOGI(TAG, "Calibrating Elevator Claw...");
//             elevator_claw.calibrate();
//         } else if (command.startsWith("e")) {
//             int elevator_pos = command.substring(1).toInt();
//             ESP_LOGI(TAG, "Moving Elevator to: %d", elevator_pos);
//             elevator_claw.move_to_position(elevator_pos);
//         }

//         else if (command.startsWith("ws")) {
//             worm_speed = command.substring(2).toInt();
//             ESP_LOGI(TAG, "Setting Worm Speed: %d Hz, Accel: %d", worm_speed, worm_accel);
//             worm_spear.set_speed(worm_speed, worm_accel);
//         } else if (command.startsWith("wa")) {
//             worm_accel = command.substring(2).toInt();
//             ESP_LOGI(TAG, "Setting Worm Speed: %d Hz, Accel: %d", worm_speed, worm_accel);
//             worm_spear.set_speed(worm_speed, worm_accel);
//         } else if (command.startsWith("wh")) {
//             worm_spear.set_home_position();
//             ESP_LOGI(TAG, "Worm home position set.");
//         } else if (command.startsWith("wp")) {
//             int pos = worm_spear.get_current_position();
//             ESP_LOGI(TAG, "Current Worm Position: %d", pos);
//         } else if (command.startsWith("wl")) {
//             bool limit_pressed = worm_spear.limit_is_pressed();
//             ESP_LOGI(TAG, "Worm limit switch pressed: %s", limit_pressed ? "true" : "false");
//         } else if (command.startsWith("wcal")) {
//             ESP_LOGI(TAG, "Calibrating Worm Spear...");
//             elevator_claw.enable_elevator();
//             worm_spear.calibrate();
//         } else if (command.startsWith("wx")) {
//             worm_spear.disable_worm();
//             ESP_LOGI(TAG, "Worm Spear disabled.");
//         } else if (command.startsWith("we")) {
//             worm_spear.enable_worm();
//             ESP_LOGI(TAG, "Worm Spear enabled.");
//         } else if (command.startsWith("w")) {
//             int spear_pos = command.substring(1).toInt();
//             ESP_LOGI(TAG, "Moving Spear to: %d", spear_pos);
//             worm_spear.move_to_position(spear_pos);
//         } else if (command == "pos") {
//             Serial.printf("Elevator Pos: %d | Worm Pos: %d\n",
//                           elevator_claw.get_current_position(),
//                           worm_spear.get_current_position());
//         } else if (command.startsWith("c")) {
//             float claw_deg = command.substring(1).toFloat();
//             ESP_LOGI(TAG, "Setting Claw to: %.2f degrees", claw_deg);
//             elevator_claw.set_claw(claw_deg);
//         } else if (command.startsWith("v")) {
//             float spear_deg = command.substring(1).toFloat();
//             worm_spear.set_spear(spear_deg, SPEAR_MOVE_DELAY);
//             ESP_LOGI(TAG, "Setting Spear to: %.2f degrees", spear_deg);
//         } else if (command.startsWith("assemble")) {
//             ESP_LOGI(TAG, "Starting tower assembly sequence...");
//             elevator_claw.set_claw(CLAW_OPEN_DEG);
//             worm_spear.set_spear(SPEAR_DOWN_DEG, 100);
//             ;
//             worm_spear.calibrate();
//             elevator_claw.calibrate();
//             elevator_claw.move_to_position(ElevatorPos::ELEV_TOWER_2_MINUS);
//             worm_spear.move_to_position(SpearPos::SPEAR_LEFT);

//             build_tower_sequence();
//             ESP_LOGI(TAG, "Tower assembly sequence completed.");
//         } else {
//             ESP_LOGW(TAG, "Unknown command: %s", command.c_str());
//         }
//         // ==========================================
//     }
//     vTaskDelay(pdMS_TO_TICKS(10));
// }

// void send_pose_setpoint(const String &command, bool relative)
// {
//     float x_m = 0.0f;
//     float y_m = 0.0f;
//     float heading_deg = 0.0f;

//     if (sscanf(command.c_str(), "%*c %f %f %f", &x_m, &y_m, &heading_deg) != 3 ||
//         !std::isfinite(x_m) || !std::isfinite(y_m) || !std::isfinite(heading_deg)) {
//         ESP_LOGW(TAG, "use: p <x_m> <y_m> <heading_deg> or r <dx_m> <dy_m> <dheading_deg>");
//         return;
//     }

//     robot_DriveCommand pose_cmd = robot_DriveCommand_init_zero;
//     pose_cmd.which_command = robot_DriveCommand_pose_tag;
//     pose_cmd.command.pose.x_m = x_m;
//     pose_cmd.command.pose.y_m = y_m;
//     pose_cmd.command.pose.theta_rad = radians(heading_deg);
//     pose_cmd.command.pose.relative = relative;

//     if (send_drive_command(pose_cmd) != ESP_OK) {
//         ESP_LOGE(TAG, "failed to send pose setpoint");
//         return;
//     }

//     ESP_LOGI(TAG,
//              "%s pose setpoint: x=%.4f m y=%.4f m heading=%.4f deg",
//              relative ? "relative" : "absolute",
//              x_m,
//              y_m,
//              heading_deg);
// }

// bool parse_speed(const String &command, const char *prefix, float *out_speed)
// {
//     if (!command.startsWith(prefix) || out_speed == nullptr) return false;

//     const String value = command.substring(strlen(prefix));
//     char trailing = '\0';
//     float speed = 0.0f;
//     if (sscanf(value.c_str(), "%f %c", &speed, &trailing) != 1 || !std::isfinite(speed) ||
//         speed < 0.0f || speed > 100.0f) {
//         ESP_LOGW(TAG, "speed must be between 0 and 100 percent");
//         return false;
//     }

//     *out_speed = speed;
//     return true;
// }

// void loop()
// {
//     if (Serial.available() <= 0) {
//         delay(10);
//         return;
//     }

//     String command = Serial.readStringUntil('\n');
//     command.trim();

//     float speed = 0.0f;
//     if (command.startsWith("p ")) {
//         send_pose_setpoint(command, false);
//     } else if (command.startsWith("r ")) {
//         send_pose_setpoint(command, true);
//     } else if (parse_speed(command, "forward ", &speed)) {
//         send_velocity(0.0f, speed);
//     } else if (parse_speed(command, "backward ", &speed)) {
//         send_velocity(0.0f, -speed);
//     } else if (parse_speed(command, "strafe left ", &speed)) {
//         send_velocity(-speed, 0.0f);
//     } else if (parse_speed(command, "strafe right ", &speed)) {
//         send_velocity(speed, 0.0f);
//     } else if (parse_speed(command, "turn left ", &speed)) {
//         send_velocity(0.0f, 0.0f, speed);
//     } else if (parse_speed(command, "turn right ", &speed)) {
//         send_velocity(0.0f, 0.0f, -speed);
//     } else if (command == "stop" || command == "s" || command == "x") {
//         send_stop();
//     } else if (command == "?") {
//         ESP_LOGI(TAG, "g <x|y|h> <kp> <ki> <kd>");
//         ESP_LOGI(TAG, "p <x_m> <y_m> <heading_deg>  (absolute pose)");
//         ESP_LOGI(TAG, "r <dx_m> <dy_m> <dheading_deg> (relative pose)");
//         ESP_LOGI(TAG, "forward <percent>");
//         ESP_LOGI(TAG, "backward <percent>");
//         ESP_LOGI(TAG, "strafe left <percent>");
//         ESP_LOGI(TAG, "strafe right <percent>");
//         ESP_LOGI(TAG, "turn left <percent>");
//         ESP_LOGI(TAG, "turn right <percent>");
//         ESP_LOGI(TAG, "stop                           (aliases: s, x)");
//     } else if (!command.isEmpty()) {
//         ESP_LOGW(TAG, "unknown command; enter ? for help");
//     }
// }
