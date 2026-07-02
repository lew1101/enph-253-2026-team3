#include <Arduino.h>
#include "control/drivetrain.hpp"

control::Drivetrain::Drivetrain(const Config &config)
    : _config(config)
{
}

bool control::Drivetrain::init()
{
    if (_config.timer == nullptr) return false;

    front_right_motor = driver::DCDriver(_config.front_right_motor_config);
    front_left_motor = driver::DCDriver(_config.front_left_motor_config);
    back_right_motor = driver::DCDriver(_config.back_right_motor_config);
    back_left_motor = driver::DCDriver(_config.back_left_motor_config);

    if (!front_right_motor.init()) return false;
    if (!front_left_motor.init()) return false;
    if (!back_right_motor.init()) return false;
    if (!back_left_motor.init()) return false;

    return true;
}

bool control::Drivetrain::forward(float speed_percentage)
{
    if (speed_percentage > 0.0) {
        front_right_motor.turn_c_clockwise();
        back_right_motor.turn_c_clockwise();
        front_left_motor.turn_clockwise();
        back_left_motor.turn_clockwise();
    }

    else {
        front_right_motor.turn_clockwise();
        back_right_motor.turn_clockwise();
        front_left_motor.turn_c_clockwise();
        back_left_motor.turn_c_clockwise();
    }

    speed_percentage = std::abs(speed_percentage);
    front_right_motor.set_speed(speed_percentage);
    back_right_motor.set_speed(speed_percentage);
    front_left_motor.set_speed(speed_percentage);
    back_left_motor.set_speed(speed_percentage);

    return true;
}

bool control::Drivetrain::strafe(float speed_percentage) // right is positive, left is negative
{
    if (speed_percentage > 0.0) {
        front_right_motor.turn_c_clockwise();
        back_right_motor.turn_clockwise();
        front_left_motor.turn_c_clockwise();
        back_left_motor.turn_clockwise();
    }

    else {
        front_right_motor.turn_clockwise();
        back_right_motor.turn_c_clockwise();
        front_left_motor.turn_clockwise();
        back_left_motor.turn_c_clockwise();
    }

    speed_percentage = std::abs(speed_percentage);
    front_right_motor.set_speed(speed_percentage);
    back_right_motor.set_speed(speed_percentage);
    front_left_motor.set_speed(speed_percentage);
    back_left_motor.set_speed(speed_percentage);

    return true;
}

bool control::Drivetrain::stop()
{
    if (!front_right_motor.stop()) return false;
    if (!front_left_motor.stop()) return false;
    if (!back_right_motor.stop()) return false;
    if (!back_left_motor.stop()) return false;

    return true;
}

bool control::Drivetrain::turn(float speed_percentage)
{
    if (speed_percentage > 0.0) {
        front_right_motor.turn_c_clockwise();
        back_right_motor.turn_c_clockwise();
        front_left_motor.turn_clockwise();
        back_left_motor.turn_clockwise();
    }

    else {
        front_right_motor.turn_clockwise();
        back_right_motor.turn_clockwise();
        front_left_motor.turn_c_clockwise();
        back_left_motor.turn_c_clockwise();
    }

    speed_percentage = std::abs(speed_percentage);
    front_right_motor.set_speed(speed_percentage);
    back_right_motor.set_speed(speed_percentage);
    front_left_motor.set_speed(speed_percentage);
    back_left_motor.set_speed(speed_percentage);

    return true;
}