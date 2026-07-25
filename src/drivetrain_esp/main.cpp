// #include "freertos/idf_additions.h"
// #include "tasks/drive.hpp"
// #include "tasks/tape_sense.hpp"
// #include "tasks/imu.hpp"
// #include "tasks/uart.hpp"

// TaskHandle_t uart_tx_handle;
// TaskHandle_t uart_rx_handle;
// TaskHandle_t drive_handle;
// TaskHandle_t imu_sensor_handle;
// TaskHandle_t tape_handle;

// void setup()
// {
//     ESP_ERROR_CHECK_WITHOUT_ABORT(start_uart_tasks(&uart_tx_handle, &uart_rx_handle));
//     ESP_ERROR_CHECK_WITHOUT_ABORT(start_imu_task(&imu_sensor_handle));
//     ESP_ERROR_CHECK_WITHOUT_ABORT(start_drive_task(&drive_handle));
//     ESP_ERROR_CHECK_WITHOUT_ABORT(start_tape_sense_task(&tape_handle));

//     vTaskDelete(nullptr);
// }

// void loop() {}

#include <Arduino.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Include your new architecture headers
#include "tasks/drive.hpp"
#include "tasks/tape_sense.hpp" 
#include "control/tape_pid.hpp"
#include "drive.pb.h"

// Expose the global PID object from drive.cpp so we can tune it via Serial
extern control::TapePID tape_pid;

// Global tuning variables
volatile float y_speed = 12.0f;    // Forward speed (vy)
volatile float x_speed = 0.0f;     // Strafe speed (vx)
volatile float omega_speed = 0.0f; // Rotational speed (omega)

float kp_y = 0.0f;
float ki_y = 0.0f;
float kd_y = 0.0f;
uint32_t cmd_sequence = 0;

void setup()
{
    Serial.begin(115200);
    start_drive_task(nullptr);
    start_tape_sense_task(nullptr); 
}

void loop()
{
    if (Serial.available() > 0) {
        String command = Serial.readStringUntil('\n');
        command.trim();

        if (command == "t") {
            robot_DriveCommand tape_cmd = robot_DriveCommand_init_zero;
            tape_cmd.sequence = ++cmd_sequence;
            tape_cmd.which_command = robot_DriveCommand_tape_follow_tag;
            tape_cmd.command.tape_follow.forward_speed_percent = y_speed;
            
            send_drive_cmd(tape_cmd);
            ESP_LOGI("Main", "Tape Following @ %.2f speed", y_speed);
        }
        else if (command == "x") {
            robot_DriveCommand stop_cmd = robot_DriveCommand_init_zero;
            stop_cmd.sequence = ++cmd_sequence;
            stop_cmd.which_command = robot_DriveCommand_stop_tag;
            stop_cmd.command.stop.brake = true;
            
            send_drive_cmd(stop_cmd);
            ESP_LOGI("Main", "Stopped");
        }
        else if (command == "w") {
            robot_DriveCommand velocity_cmd = robot_DriveCommand_init_zero;
            velocity_cmd.sequence = ++cmd_sequence;
            velocity_cmd.which_command = robot_DriveCommand_velocity_tag;
            velocity_cmd.command.velocity.vx_percent = x_speed;
            velocity_cmd.command.velocity.vy_percent = y_speed;
            velocity_cmd.command.velocity.omega_percent = omega_speed;

            send_drive_cmd(velocity_cmd);
            ESP_LOGI("Main", "Velocity Command: vx=%.2f, vy=%.2f, omega=%.2f", x_speed, y_speed, omega_speed);
        }
        else if (command.startsWith("v")) {
            // Parse the command for vx, vy, and omega
            int first_space = command.indexOf(' ');
            int second_space = command.indexOf(' ', first_space + 1);

            if (first_space != -1 && second_space != -1) {
                x_speed = command.substring(first_space + 1, second_space).toFloat();
                y_speed = command.substring(second_space + 1).toFloat();
                // Assuming omega is optional and defaults to 0 if not provided
                omega_speed = (second_space + 1 < command.length()) ? command.substring(second_space + 1).toFloat() : 0.0f;

                ESP_LOGI("Main", "Updated speeds: vx=%.2f, vy=%.2f, omega=%.2f", x_speed, y_speed, omega_speed);
            } else {
                ESP_LOGW("Main", "Invalid velocity command format. Use: v <vx> <vy> [<omega>]");
            }
        }
        else if (command.startsWith("1") || command.startsWith("P")) {
            kp_y = command.substring(1).toFloat();
            tape_pid.set_gain(kp_y, ki_y, kd_y);
            ESP_LOGI("Supervisor", "Kp explicitly set to: %.2f", kp_y);
        }
        else if (command.startsWith("2") || command.startsWith("I")) {
            ki_y = command.substring(1).toFloat();
            tape_pid.set_gain(kp_y, ki_y, kd_y);
            ESP_LOGI("Supervisor", "Ki explicitly set to: %.2f", ki_y);
        }
        else if (command.startsWith("3") || command.startsWith("D")) {
            kd_y = command.substring(1).toFloat();
            tape_pid.set_gain(kp_y, ki_y, kd_y);
            ESP_LOGI("Supervisor", "Kd explicitly set to: %.2f", kd_y);
        }
        else if (command.startsWith("f") || command.startsWith("F")) {
            y_speed = command.substring(1).toFloat();
            ESP_LOGI("Supervisor", "Tape follow speed explicitly set to: %.2f", y_speed);
        }
        else if (command.startsWith("a")) {
            x_speed = command.substring(1).toFloat();
            ESP_LOGI("Supervisor", "Strafe speed explicitly set to: %.2f", x_speed);
        }
        else if (command.startsWith("r")) {
            omega_speed = command.substring(1).toFloat();
            ESP_LOGI("Supervisor", "Rotational speed explicitly set to: %.2f", omega_speed);
        }
    }
}