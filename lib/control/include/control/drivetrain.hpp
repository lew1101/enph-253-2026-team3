#include <Arduino.h>
#include "actuators/dc_driver.hpp"

namespace control {
class Drivetrain {
    public:
        struct Config{
            mcpwm_timer_handle_t timer = nullptr;

            driver::DCDriver::Config front_right_motor_config;
            driver::DCDriver::Config front_left_motor_config;
            driver::DCDriver::Config back_right_motor_config;
            driver::DCDriver::Config back_left_motor_config;
        };

        Drivetrain() = default;

        explicit Drivetrain(const Config &config); 

        bool init();
        
        bool forward(float speed_percentage); // 0% to 100%
        bool strafe(float speed_percentage); // 0% to 100%
        bool stop();

        bool turn(float speed_percentage); // 0% to 100%, positive is clockwise, negative is counter-clockwise

    private:
        Config _config;
        double wheel_radius = 0.05; // meters
        double wheel_base = 0.2; // ditance between front and rear (meters)
        double track_width = 0.2; // distance between left and right (meters)

        driver::DCDriver front_right_motor;
        driver::DCDriver front_left_motor;
        driver::DCDriver back_right_motor;
        driver::DCDriver back_left_motor;
    };
} // namespace control