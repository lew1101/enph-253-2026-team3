#include "drive.pb.h"
#include "freertos/idf_additions.h"

#include "esp_err.h"
#include "esp_log.h"

#include "actuators/limit.hpp"

#include "portmacro.h"
#include "projdefs.h"
#include "shared/robot_flags.hpp"
#include "supervisor.hpp"

#include "tasks/uart.hpp"
#include "tasks/camera_uart.hpp"
#include "tasks/metal.hpp"

#include "front_chassis.hpp"

DebouncedLimitSwitch worm_limit{GPIO_NUM_10};

void setup()
{
    Serial.begin(115200);
    delay(1000);

    supervisor::init();
    supervisor::attach_main_loop();

    ESP_ERROR_CHECK(start_master_uart_tasks());

    ESP_ERROR_CHECK(front_chassis_init());
    ESP_ERROR_CHECK_WITHOUT_ABORT(start_metal_detector_task());

    const esp_err_t camera_uart_err = start_camera_uart_task();
    log_i("camera UART task start: %s", esp_err_to_name(camera_uart_err));
    delay(1000);

    worm_spear.calibrate();
    elevator_claw.calibrate();
}

void rock_and_teletubbies() {

}

enum ElevatorPos : int32_t {
    ELEV_FLOOR = 0,
    ELEV_TOWER_1 = 1,
    ELEV_TOWER_2_MINUS = 2,
    ELEV_TOWER_2 = 3,
    ELEV_TOWER_2_PLUS = 4,
    ELEV_TOWER_3 = 6,
    ELEV_TOWER_3_PLUS = 7,
    ELEV_BACK = 8,
};

enum SpearPos : int32_t { //
    SPEAR_LEFT = 50'000,
    SPEAR_CENTRE = 25'000,
    SPEAR_RIGHT = 0,
    SPEAR_CENTERING = 35'000
};

constexpr float SPEAR_UP_DEG = 90.0f;
constexpr float SPEAR_45_DEG = 45.0f;
constexpr float SPEAR_DOWN_DEG = 0.0f;

constexpr float CLAW_OPEN_DEG = 0.0f;
constexpr float CLAW_CLOSE_TOWER_DEG = 20.0f;
constexpr float CLAW_CLOSE_ROCK_DEG = 70.0f;

void assemble_tower()
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

    robot_DriveCommand forward = robot_DriveCommand_init_zero;
    forward.which_command = robot_DriveCommand_pose_tag;
    forward.command.pose.relative = true;
    forward.command.pose.x_m = 0.0f;
    forward.command.pose.y_m = Y_OFFSET;
    forward.command.pose.theta_rad = 0.0f;

    robot_DriveCommand backward = robot_DriveCommand_init_zero;
    backward.which_command = robot_DriveCommand_pose_tag;
    backward.command.pose.relative = true;
    backward.command.pose.x_m = 0.0f;
    backward.command.pose.y_m = -Y_OFFSET;
    backward.command.pose.theta_rad = 0.0f;

    xEventGroupClearBits(supervisor::g_robot_control_flags, robot_flags::CONTROL_ACTUATORS);
    xEventGroupSetBits(supervisor::g_robot_control_flags, robot_flags::CONTROL_DRIVE_ENABLED);

    // PIECE 1
    worm_spear.spear_angle(SPEAR_DOWN_DEG);
    delay(1000);

    elevator_claw.move_to_position(ElevatorPos::ELEV_TOWER_1);
    elevator_claw.set_claw(CLAW_OPEN_DEG);
    worm_spear.move_to_position(SpearPos::SPEAR_LEFT);
    send_drive_command(forward);
    supervisor::wait_for_notification(robot_flags::NOTIFY_DRIVE_TARGET_REACHED);

    worm_spear.spear_angle(SPEAR_UP_DEG);
    delay(1500);
    elevator_claw.set_claw(CLAW_CLOSE_TOWER_DEG);
    delay(1200);
    send_drive_command(backward);
    delay(500);
    worm_spear.move_to_position(SpearPos::SPEAR_CENTRE);
    elevator_claw.move_to_position(ElevatorPos::ELEV_TOWER_2_PLUS);
    delay(1000);
    supervisor::wait_for_notification(robot_flags::NOTIFY_DRIVE_TARGET_REACHED);

    // PIECE 2
    send_drive_command(forward);
    supervisor::wait_for_notification(robot_flags::NOTIFY_DRIVE_TARGET_REACHED);

    worm_spear.spear_angle(SPEAR_UP_DEG);
    delay(1500);
    elevator_claw.move_to_position(ElevatorPos::ELEV_TOWER_2);
    delay(1000);
    elevator_claw.set_claw(CLAW_OPEN_DEG);
    delay(1500);
    elevator_claw.move_to_position(ElevatorPos::ELEV_TOWER_2_MINUS);
    delay(1500);
    elevator_claw.set_claw(CLAW_CLOSE_TOWER_DEG);
    delay(1500);
    elevator_claw.move_to_position(ElevatorPos::ELEV_TOWER_3_PLUS);
    send_drive_command(backward);
    delay(1000);
    worm_spear.spear_angle(SPEAR_DOWN_DEG);
    worm_spear.move_to_position(SpearPos::SPEAR_RIGHT);
    delay(1000);
    supervisor::wait_for_notification(robot_flags::NOTIFY_DRIVE_TARGET_REACHED);

    // PIECE 3
    send_drive_command(forward);
    supervisor::wait_for_notification(robot_flags::NOTIFY_DRIVE_TARGET_REACHED);

    worm_spear.spear_angle(SPEAR_UP_DEG);
    delay(1500);
    elevator_claw.move_to_position(ElevatorPos::ELEV_TOWER_3);
    delay(1000);
    elevator_claw.set_claw(CLAW_OPEN_DEG);
    delay(1500);
    elevator_claw.move_to_position(ElevatorPos::ELEV_TOWER_2_MINUS);
    delay(1500);
    elevator_claw.set_claw(CLAW_CLOSE_TOWER_DEG);
    delay(1500);
    elevator_claw.move_to_position(ElevatorPos::ELEV_TOWER_2_PLUS);
    delay(1000);
    worm_spear.move_to_position(SpearPos::SPEAR_CENTERING);
    send_drive_command(backward);
    supervisor::wait_for_notification(robot_flags::NOTIFY_DRIVE_TARGET_REACHED);
}

static int elev_speed = 9000;
static int elev_accel = 1600;
static int worm_speed = 9000;
static int worm_accel = 1600;

void loop()
{
    // PHASE 1: ROCKS and TELETUBBY



    // PHASE 2: TOWER
    //===============
    // Reorient to the tower:
    /* Tower tape detected -> forward and back until tape aligned with side sensors -> rotate CCW
    until side tape aligned with front sensors */

    if (Serial.available()) {
        String command = Serial.readStringUntil('\n');
        command.trim();
        if (command.length() == 0) return;

        // ELEVATOR COMMANDS
        if (command.startsWith("es")) {
            elev_speed = command.substring(2).toInt();
            ESP_LOGI("Main", "Setting Elevator Speed: %d Hz, Accel: %d", elev_speed, elev_accel);
            elevator_claw.set_speed(elev_speed, elev_accel);
        }
        else if (command.startsWith("ea")) {
            elev_accel = command.substring(2).toInt();
            ESP_LOGI("Main", "Setting Elevator Speed: %d Hz, Accel: %d", elev_speed, elev_accel);
            elevator_claw.set_speed(elev_speed, elev_accel);
        }
        else if (command.startsWith("eh")) {
            elevator_claw.set_home_position();
            ESP_LOGI("Main", "Elevator home position set.");
        }
        else if (command.startsWith("ep")) {
            int pos = elevator_claw.get_current_position();
            ESP_LOGI("Main", "Current Elevator Position: %d", pos);
        }
        else if (command.startsWith("e")) {
            int elevator_pos = command.substring(1).toInt(); 
            ESP_LOGI("Main", "Moving Elevator to: %d", elevator_pos);
            elevator_claw.move_to_position(elevator_pos);
        }

        // WORM SPEAR COMMANDS
        else if (command.startsWith("ws")) {
            worm_speed = command.substring(2).toInt();
            ESP_LOGI("Main", "Setting Worm Speed: %d Hz, Accel: %d", worm_speed, worm_accel);
            worm_spear.set_speed(worm_speed, worm_accel);
        }
        else if (command.startsWith("wa")) {
            worm_accel = command.substring(2).toInt();
            ESP_LOGI("Main", "Setting Worm Speed: %d Hz, Accel: %d", worm_speed, worm_accel);
            worm_spear.set_speed(worm_speed, worm_accel);
        }
        else if (command.startsWith("wh")) {
            worm_spear.set_home_position();
            ESP_LOGI("Main", "Worm home position set.");
        }
        else if (command.startsWith("wp")) {
            int pos = worm_spear.get_current_position();
            ESP_LOGI("Main", "Current Worm Position: %d", pos);
        }
        else if (command.startsWith("w")) {
            int spear_pos = command.substring(1).toInt();
            ESP_LOGI("Main", "Moving Spear to: %d", spear_pos);
            worm_spear.move_to_position(spear_pos);
        }

        // UTILITY & SERVO COMMANDS
        else if (command == "pos") {
            Serial.printf("Elevator Pos: %d | Worm Pos: %d\n", 
                        elevator_claw.get_current_position(), 
                        worm_spear.get_current_position());
        }
        else if (command.startsWith("c")) {
            float claw_deg = command.substring(1).toFloat();
            ESP_LOGI("Main", "Setting Claw to: %.2f degrees", claw_deg);
            elevator_claw.set_claw(claw_deg);
        }
        else if (command.startsWith("v")) {
            float spear_deg = command.substring(1).toFloat();
            worm_spear.spear_angle(spear_deg);
            ESP_LOGI("Main", "Setting Spear to: %.2f degrees", spear_deg);
        }
                else if (command.startsWith("assemble")) {
                    ESP_LOGI("Main", "Starting tower assembly sequence...");
                    assemble_tower();
                    ESP_LOGI("Main", "Tower assembly sequence completed.");
                }
                else {
                    ESP_LOGW("Main", "Unknown command: %s", command.c_str());
                }
                // ==========================================
            }

            input_buffer = "";
            prompt_printed = false; // Trigger a new "> " prompt
        }

        else {
            input_buffer += c;
            Serial.print(c); // Echo the character back so you can see it!
        }
    }
    vTaskDelay(pdMS_TO_TICKS(10));

    // PHASE 3: SOLAR PANEL

//     vTaskDelay(portMAX_DELAY);
}
