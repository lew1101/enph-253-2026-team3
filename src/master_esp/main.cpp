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

static bool worm_calibrated = false;
static bool elev_calibrated = false;

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

constexpr float SPEAR_UP_DEG = 90.0f;
constexpr float SPEAR_UP_SLIGHTLY = 10.0f;
constexpr float SPEAR_DOWN_DEG = 5.0f;
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
    constexpr float Y_OFFSET = 0.08f; // m

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
    xEventGroupSetBits(supervisor::g_robot_control_flags, robot_flags::CONTROL_DRIVE_ENABLED);
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
    xEventGroupSetBits(supervisor::g_robot_control_flags, robot_flags::CONTROL_DRIVE_ENABLED);

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

    elevator_claw.set_claw(CLAW_OPEN_DEG);
    worm_spear.set_spear(SPEAR_DOWN_DEG, 100.0f);

    if (worm_spear.calibrate() != ESP_OK) {
        ESP_LOGE(TAG, "worm_spear calibration sequence failed... ");
        worm_calibrated = false;
    } else {
        worm_calibrated = true;
    }

    if (elevator_claw.calibrate() != ESP_OK) {
        ESP_LOGE(TAG, "elevator_claw calibration sequence failed... ");
        elev_calibrated = false;
    } else {
        elev_calibrated = true;
    }

    elevator_claw.move_to_position(ElevatorPos::ELEV_TOWER_2_MINUS);
} // namespace

void loop()
{
    xEventGroupSetBits(supervisor::g_robot_control_flags, robot_flags::CONTROL_DRIVE_ENABLED);

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

// void send_velocity(float vx_percent, float vy_percent, float omega_percent = 0.0f)
// {
//     robot_DriveCommand velocity_cmd = robot_DriveCommand_init_zero;
//     velocity_cmd.which_command = robot_DriveCommand_velocity_tag;
//     velocity_cmd.command.velocity.vx_percent = vx_percent;
//     velocity_cmd.command.velocity.vy_percent = vy_percent;
//     velocity_cmd.command.velocity.omega_percent = omega_percent;

//     if (send_drive_command(velocity_cmd) == ESP_OK) {
//         ESP_LOGI(
//             TAG, "velocity: x=%.2f%% y=%.2f%% turn=%.2f%%", vx_percent, vy_percent,
//             omega_percent);
//     } else {
//         ESP_LOGE(TAG, "failed to send velocity command");
//     }
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
// void send_stop()
// {
//     robot_DriveCommand stop_cmd = robot_DriveCommand_init_zero;
//     stop_cmd.which_command = robot_DriveCommand_stop_tag;
//     stop_cmd.command.stop.brake = true;

//     if (send_drive_command(stop_cmd) == ESP_OK) {
//         ESP_LOGI(TAG, "stopped");
//     } else {
//         ESP_LOGE(TAG, "failed to send stop command");
//     }
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
