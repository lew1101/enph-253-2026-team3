#include "freertos/idf_additions.h"

#include "esp_err.h"
#include "esp_log.h"

#include "actuators/limit.hpp"

#include "supervisor.hpp"

#include "tasks/uart.hpp"
#include "tasks/camera_uart.hpp"
#include "tasks/metal.hpp"

#include "front_chassis.hpp"

DebouncedLimitSwitch elev_limit{GPIO_NUM_8};
DebouncedLimitSwitch worm_limit{GPIO_NUM_10};

void setup()
{
    Serial.begin(115200);
    delay(1000);

    supervisor::init();
    supervisor::attach_main_loop();

    ESP_ERROR_CHECK(start_master_uart_tasks());
    ESP_ERROR_CHECK_WITHOUT_ABORT(front_chassis_init());
    // ESP_ERROR_CHECK_WITHOUT_ABORT(start_metal_detector_task());
    // const esp_err_t camera_uart_err = start_camera_uart_task();
    // log_i("camera UART task start: %s", esp_err_to_name(camera_uart_err));
    // delay(1000);

    // worm_spear.calibrate();
    // elevator_claw.calibrate();
    ESP_LOGI("Main", "WormSpear and ElevatorClaw calibrated successfully");
}

enum ElevatorPos : int32_t {
    ELEV_FLOOR = 0,
    ELEV_TOWER_UP = 1,
    ELEV_TOWER_STACK = 2
};

enum SpearPos : int32_t { SPEAR_LEFT = 50'000, SPEAR_CENTRE = 25'000, SPEAR_RIGHT = 0, CRESCENT_MOON = 30'000};

constexpr float SPEAR_UP = 90.0f;
constexpr float SPEAR_DOWN = 45.0f;
constexpr float CLAW_OPEN = 180.0f;
constexpr float CLAW_CLOSED_ROCK = 65.0f;
constexpr float CLAW_CLOSED_TOWER = 20.0f;

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

    // FIRST TOWER PIECE
    // move forward
    elevator_claw.set_claw(CLAW_OPEN);
    worm_spear.spear_angle(SPEAR_UP);
    delay(200);

    // move backward

    worm_spear.move_to_position(SPEAR_CENTRE);
    while(!worm_spear.worm_done()) { vTaskDelay(1); }
    elevator_claw.move_to_position(ELEV_FLOOR);
    while(!elevator_claw.elevator_done()) { vTaskDelay(1); }
    elevator_claw.set_claw(CLAW_CLOSED_TOWER);
    elevator_claw.move_to_position(ELEV_TOWER_UP);
    while(!elevator_claw.elevator_done()) { vTaskDelay(1); }
    worm_spear.spear_angle(SPEAR_DOWN);

    // SECOND TOWER PIECE
    // move forward
    worm_spear.spear_angle(SPEAR_UP); 
    delay(200);

    //move backward
    worm_spear.move_to_position(SPEAR_CENTRE);
    while(!worm_spear.worm_done()) { vTaskDelay(1); }

    elevator_claw.move_to_position(ELEV_TOWER_STACK);
    while(!elevator_claw.elevator_done()) { vTaskDelay(1); }
    elevator_claw.set_claw(CLAW_OPEN);
    delay(200);
    elevator_claw.move_to_position(ELEV_FLOOR);
    while(!elevator_claw.elevator_done()) { vTaskDelay(1); }
    elevator_claw.set_claw(CLAW_CLOSED_TOWER);
    delay(200);
    elevator_claw.move_to_position(ELEV_TOWER_UP);
    while(!elevator_claw.elevator_done()) { vTaskDelay(1); }
    worm_spear.spear_angle(SPEAR_DOWN);

    // THIRD TOWER PIECE
    worm_spear.move_to_position(SPEAR_RIGHT);
    while(!elevator_claw.elevator_done()) { vTaskDelay(1); }

    // move forward
    worm_spear.spear_angle(SPEAR_UP);
    elevator_claw.move_to_position(ELEV_TOWER_STACK);
    while(!elevator_claw.elevator_done()) { vTaskDelay(1); }
    elevator_claw.set_claw(CLAW_OPEN);
    delay(200);
    elevator_claw.move_to_position(ELEV_FLOOR);
    while(!elevator_claw.elevator_done()) { vTaskDelay(1); }
    elevator_claw.set_claw(CLAW_CLOSED_TOWER);
    delay(200);
    elevator_claw.move_to_position(ELEV_TOWER_UP);
    while(!elevator_claw.elevator_done()) { vTaskDelay(1); }
    worm_spear.spear_angle(SPEAR_DOWN);
    worm_spear.move_to_position(CRESCENT_MOON);

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
    }

    // Keep the RTOS watchdog happy
    vTaskDelay(pdMS_TO_TICKS(10));

    // PHASE 3: SOLAR PANEL

}