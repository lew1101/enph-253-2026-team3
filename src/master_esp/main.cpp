// #include <Arduino.h>

// #include "freertos/idf_additions.h"
// #include "projdefs.h"
// #include "tasks/metal.hpp"
// #include "tasks/uart.hpp"
// #include "supervisor.hpp"

// #include "shared/robot_flags.hpp"

// #include "control/elevator_claw.hpp"
// #include "control/worm_spear.hpp"

// control::ElevatorClaw elev_claw;
// control::WormSpear worm_spear;

// enum ElevPos : int32_t { FLOOR = 0, TOWER_1 = 0, TOWER_2 = 0, TOWER_3 = 0, OVER_BACK = 0 };

// void setup()
// {
//     supervisor::init();
//     supervisor::attach_main_loop();

//     ESP_ERROR_CHECK(start_master_uart_tasks());
//     ESP_ERROR_CHECK(start_metal_detector_task());
// }

// void assemble_tower()
// {
//     // ===============
//     // Reorient to the tower:
//     /* Tower tape detected -> forward and back until tape aligned with side sensors -> rotate CCW
//     until side tape aligned with front sensors */

//     // ===============
//     // Built the tower
//     /*
//     -> Forward -> Spear up -> Backward and center worm ->
//     Elevator down -> Claw close -> Elevator up -> Spear down -> Forward
//     -> Spear up -> Backward and center worm -> Elevator down -> Claw close -> Elevator up ->
//     Spear down
//     -> Worm to third tower piece -> Forward
//     -> Spear up -> Backward and center worm -> Elevator down -> Claw close -> Elevator up ->
//     Spear down
//     -> Worm center cresent moon while driving backward -> rotate CW until find tape -> Tape
//     follow
//     -> Boss tape detected -> forward and back until tape aligned with side sensors -> rotate CCW
//     until side tape aligned with front sensors -> Drive forward slowly (cresent moon will auto
//     align the robot with the boss) -> Elevator down -> Claw open */

// }

// void loop()
// {
//     xEventGroupSetBits(supervisor::g_robot_control_flags, robot_flags::CONTROL_DRIVE_ENABLED);

//     assemble_tower();
//     // yay finished
//     vTaskDelay(portMAX_DELAY);
// }

#include <Arduino.h>
#include "esp_log.h"
#include "actuators/servo.hpp"
#include "control/worm_spear.hpp"
#include "control/elevator_claw.hpp"
#include "FastAccelStepper.h"
#include "tasks/camera_uart.hpp"

#define SPEAR_SERVO_PIN GPIO_NUM_6
#define CLAW_SERVO_PIN GPIO_NUM_5
#define WORM_STEP_PIN GPIO_NUM_15
#define WORM_DIR_PIN GPIO_NUM_16
#define WORM_CALIBRATION_SWITCH_PIN GPIO_NUM_12
#define ELEVATOR_STEP_PIN GPIO_NUM_42
#define ELEVATOR_DIR_PIN GPIO_NUM_41
#define ELEVATOR_CALIBRATION_SWITCH_PIN GPIO_NUM_10

driver::ServoDriver::Config spear_servo_config{.gpio = SPEAR_SERVO_PIN, // Specify your pin here
                                               .channel = 1,
                                               .freq_hz = 50,
                                               .duty_res_bits = 14,
                                               .min_pulse_us = 500,
                                               .max_pulse_us = 2400,
                                               .min_pulse_deg = 0.0f,
                                               .max_pulse_deg = 180.0f,
                                               .min_clamp_deg = 0.0f,
                                               .max_clamp_deg = 180.0f};

driver::ServoDriver::Config claw_servo_config{.gpio = CLAW_SERVO_PIN, // Specify your pin here
                                              .channel = 2,
                                              .freq_hz = 50,
                                              .duty_res_bits = 14,
                                              .min_pulse_us = 500,
                                              .max_pulse_us = 2400,
                                              .min_pulse_deg = 0.0f,
                                              .max_pulse_deg = 180.0f,
                                              .min_clamp_deg = 0.0f,
                                              .max_clamp_deg = 180.0f};

FastAccelStepperEngine engine = FastAccelStepperEngine();

control::WormSpear::Config worm_config{
    .engine = nullptr, // Set this to your FastAccelStepperEngine instance
    .worm_step_pin = WORM_STEP_PIN,
    .worm_dir_pin = WORM_DIR_PIN,
    .worm_calibration_switch_pin = WORM_CALIBRATION_SWITCH_PIN,
    .speed_hz = 9000,
    .acceleration_hz_per_s = 1600,
    .spear_servo_config = spear_servo_config};

control::ElevatorClaw::Config elevator_config{
    .engine = nullptr, // Set this to your FastAccelStepperEngine instance
    .elevator_step_pin = ELEVATOR_STEP_PIN,
    .elevator_dir_pin = ELEVATOR_DIR_PIN,
    .elevator_calibration_switch_pin = ELEVATOR_CALIBRATION_SWITCH_PIN,
    .speed_hz = 9000,
    .acceleration_hz_per_s = 1600,
    .claw_servo_config = claw_servo_config};

control::WormSpear worm_spear;
control::ElevatorClaw elevator_claw;

void setup()
{
    Serial.begin(115200);
    ESP_ERROR_CHECK_WITHOUT_ABORT(start_camera_uart_task());
    delay(1000);

    engine.init();
    worm_config.engine = &engine;
    elevator_config.engine = &engine;
    worm_spear = control::WormSpear(worm_config);
    elevator_claw = control::ElevatorClaw(elevator_config);
    if (worm_spear.init() != ESP_OK) {
        Serial.println("Failed to initialize WormSpear");
    }
    if (elevator_claw.init() != ESP_OK) {
        Serial.println("Failed to initialize ElevatorClaw");
    } else {
        Serial.println("WormSpear and ElevatorClaw initialized successfully");
    }
}
void loop()
{
    if (Serial.available() > 0) {
        String command = Serial.readStringUntil('\n');
        command.trim(); // Remove newline/spaces

        if (command == "u") {
            Serial.println("spear up");
            worm_spear.spear_up();
        } else if (command == "d") {
            Serial.println("spear down");
            worm_spear.spear_down();
        } else if (command == "o") {
            Serial.println("claw open");
            elevator_claw.open_claw_tower();
        } else if (command == "c") {
            Serial.println("claw close");
            elevator_claw.close_claw();
        } else if (command == "r") {
            Serial.println("rock claw open");
            elevator_claw.open_claw_rock();
        } else if (command.startsWith("w")) {
            float worm_step = command.substring(1).toFloat();
            Serial.printf("Moving worm to step: %.2f\n", worm_step);
            worm_spear.move_to_position(worm_step);
        } else if (command.startsWith("e")) {
            float elevator_step = command.substring(1).toFloat();
            Serial.printf("Moving elevator to step: %.2f\n", elevator_step);
            elevator_claw.move_to_position(-elevator_step);
        }
    }
}
