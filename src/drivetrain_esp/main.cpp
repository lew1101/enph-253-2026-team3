#include <Arduino.h>
#include "tasks/drive.hpp"
#include "tasks/tape_sense.hpp"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "control/PID.hpp"

#define FR_MOTOR_CW_PIN GPIO_NUM_45
#define FR_MOTOR_CCW_PIN GPIO_NUM_46
#define BR_MOTOR_CW_PIN GPIO_NUM_42
#define BR_MOTOR_CCW_PIN GPIO_NUM_41
#define FL_MOTOR_CW_PIN GPIO_NUM_16
#define FL_MOTOR_CCW_PIN GPIO_NUM_15
#define BL_MOTOR_CW_PIN GPIO_NUM_17
#define BL_MOTOR_CCW_PIN GPIO_NUM_18
#define FL_TAPE_PIN GPIO_NUM_1
#define FM_TAPE_PIN GPIO_NUM_2
#define FR_TAPE_PIN GPIO_NUM_3
#define BL_TAPE_PIN GPIO_NUM_4
#define BM_TAPE_PIN GPIO_NUM_5
#define BR_TAPE_PIN GPIO_NUM_6
#define L1_TAPE_PIN GPIO_NUM_7
#define L2_TAPE_PIN GPIO_NUM_8

volatile float y_speed = 0.0f; // Forward/backward speed percentage
volatile float x_speed = 0.0f;  // Strafe speed percentage
float kp = 0.0f;
float ki = 0.0f;
float kd = 0.0f;

mcpwm_timer_handle_t timer_0 = nullptr;
mcpwm_timer_handle_t timer_1 = nullptr;

using namespace control;

extern PID tape_pid;

control::Drivetrain::Config drivetrain_config = {
    .timer_0 = timer_0,
    .timer_1 = timer_1,
    .front_right_motor_config = {
        .clockwise_pwm_output = FR_MOTOR_CW_PIN,
        .c_clockwise_pwm_output = FR_MOTOR_CCW_PIN,
        .min_percentage = 30.0f,
        .clamp_percentage = 100.0f
    },
    .back_right_motor_config = {
        .clockwise_pwm_output = BR_MOTOR_CW_PIN,
        .c_clockwise_pwm_output = BR_MOTOR_CCW_PIN,
        .min_percentage = 30.0f,
        .clamp_percentage = 100.0f
    },
    .front_left_motor_config = {
        .clockwise_pwm_output = FL_MOTOR_CW_PIN,
        .c_clockwise_pwm_output = FL_MOTOR_CCW_PIN,
        .min_percentage = 30.0f,
        .clamp_percentage = 95.0f
    },
    .back_left_motor_config = {
        .clockwise_pwm_output = BL_MOTOR_CW_PIN,
        .c_clockwise_pwm_output = BL_MOTOR_CCW_PIN,
        .min_percentage = 30.0f,
        .clamp_percentage = 88.0f
    }
};

DriveTaskConfig task_cfg = { .stack_depth = 4096, .priority = 4, .core_id = 1, .period_ms = 5.0f };
TapeSenseTaskConfig tape_task_cfg = {
    .fl_tape_pin = FL_TAPE_PIN,
    .fm_tape_pin = FM_TAPE_PIN,
    .fr_tape_pin = FR_TAPE_PIN,
    .bl_tape_pin = BL_TAPE_PIN,
    .bm_tape_pin = BM_TAPE_PIN,
    .br_tape_pin = BR_TAPE_PIN,
    .l1_tape_pin = L1_TAPE_PIN,
    .l2_tape_pin = L2_TAPE_PIN,
    .high_threshold = 500,
    .low_threshold = 300,
    .stack_depth = 4096,
    .priority = 4,
    .core_id = 1,
    .period_ms = 5.0f
};

void setup()
{
    Serial.begin(115200);
    
    start_drive_task(task_cfg, drivetrain_config, nullptr);
    start_tape_sense_task(tape_task_cfg, nullptr);
}


void loop()
{
    if (Serial.available() > 0) {
        String command = Serial.readStringUntil('\n');
        command.trim();

        if (command == "t") {
            DriveCommand tape_cmd;
            tape_cmd.mode = DriveMode::TAPE_FOLLOW;
            tape_cmd.tape_follow_speed = 80.0f;
            tape_cmd.rot_speed = 0.0f; // Rotation will be handled by the PID controller in the drive task
            send_drive_cmd(tape_cmd);
            ESP_LOGI("Main", "Tape Following");
        }
        else if (command == "x") {
            DriveCommand stop_cmd;
            stop_cmd.mode = DriveMode::STOP;
            send_drive_cmd(stop_cmd);
            ESP_LOGI("Main", "Stopping");
        }
        if (command == "]") {
            kp += 1.0f;
            tape_pid.set_gain(kp, ki, kd);
            ESP_LOGI("Supervisor", "PID gains set to Kp=%.1f, Ki=%.1f, Kd=%.1f", kp, ki, kd);
        } else if (command == "[") {
            kp -= 1.0f;
            tape_pid.set_gain(kp, ki, kd);
            ESP_LOGI("Supervisor", "PID gains set to Kp=%.1f, Ki=%.1f, Kd=%.1f", kp, ki, kd);
        }
        if (command == "+") {
            kp += 0.1f;
            tape_pid.set_gain(kp, ki, kd);
            ESP_LOGI("Supervisor", "PID gains set to Kp=%.1f, Ki=%.1f, Kd=%.1f", kp, ki, kd);
        } else if (command == "-") {
            kp -= 0.1f;
            tape_pid.set_gain(kp, ki, kd);
            ESP_LOGI("Supervisor", "PID gains set to Kp=%.1f, Ki=%.1f, Kd=%.1f", kp, ki, kd);
        }
        else if (command.startsWith("p") || command.startsWith("P")) {
            // Extract everything after the 'p' and convert it to a float
            float new_kp = command.substring(1).toFloat();
            kp = new_kp;
            tape_pid.set_gain(kp, 0.0f, ki);
            ESP_LOGI("Supervisor", "Kp explicitly set to: %.2f", kp);
        }
        // Format: "i0.5" sets Ki to 0.5 (for later!)
        else if (command.startsWith("i") || command.startsWith("I")) {
            // Assuming you have a `ki` variable defined globally
            float new_ki = command.substring(1).toFloat();
            ki = new_ki;
            tape_pid.set_gain(kp, ki, kd);
            ESP_LOGI("Supervisor", "Ki explicitly set to: %.2f", ki);
        }
        else if (command.startsWith("d") || command.startsWith("D")) {
            // Extract everything after the 'd' and convert it to a float
            float new_kd = command.substring(1).toFloat();
            kd = new_kd;
            tape_pid.set_gain(kp, ki, kd);
            ESP_LOGI("Supervisor", "Kd explicitly set to: %.2f", kd);
        }
    }


    // EventBits_t flags = xEventGroupGetBits(g_robot_flags);s
    // ESP_LOGI("Supervisor", "TAPE FOLLOW TASK: %d", supervisor::has_flag(flags, RobotFlag::ROBOT_FLAG_TAPE_ACTIVE));


    // supervisor::update();
    // vTaskDelay(pdMS_TO_TICKS(20)); // 50Hz
}

