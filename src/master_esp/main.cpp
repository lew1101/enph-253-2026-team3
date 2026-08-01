#include "freertos/idf_additions.h"
#include <atomic>
#include <cmath>
#include <initializer_list>

#include "esp_err.h"
#include "esp_check.h"

#include "portmacro.h"
#include "shared/robot_flags.hpp"
#include "supervisor.hpp"
#include "drive_pose.hpp"
#include "waypoints.hpp"
#include "front_chassis.hpp"

#include "tasks/camera_uart.hpp"
#include "tasks/uart.hpp"
#include "tasks/metal.hpp"

#include "sensors/metal_detector.hpp"

using control::Pose;

static constexpr char TAG[]{"master_main"};

void setup()
{
    Serial.begin(115200);
    delay(1000);

    start_metal_detector_task();

    vTaskDelete(nullptr);
}

void loop() { vTaskDelay(portMAX_DELAY); }

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
