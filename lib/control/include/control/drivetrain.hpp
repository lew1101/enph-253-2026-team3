#include <Arduino.h>
#include "actuators/dc_driver.hpp"

namespace control {
class Drivetrain {
    public:
        struct Config{
            mcpwm_timer_handle_t timer_0 = nullptr;
            mcpwm_timer_handle_t timer_1 = nullptr;

            driver::DCDriver::Config front_right_motor_config;
            driver::DCDriver::Config back_right_motor_config;
            driver::DCDriver::Config front_left_motor_config;
            driver::DCDriver::Config back_left_motor_config;
        };

        Drivetrain() = default;

        explicit Drivetrain(const Config &config); 

        bool init();
        
        bool forward(float speed_percentage); // 0% to 100%
        bool strafe(float speed_percentage); // 0% to 100%
        bool stop();
        bool turn(float speed_percentage); // 0% to 100%, positive is clockwise, negative is counter-clockwise

        bool update();
    private:
        Config _config;
        bool stopped = true;
        int delay_time_ms = 0;

        double wheel_radius = 0.05; // meters
        double wheel_base = 0.2; // ditance between front and rear (meters)
        double track_width = 0.2; // distance between left and right (meters)

        int timer_resolution_hz = 1000000;
        int PWM_frequency_hz = 10000;
        float period_tick = (timer_resolution_hz / (PWM_frequency_hz * 2));

        driver::DCDriver front_right_motor;
        driver::DCDriver front_left_motor;
        driver::DCDriver back_right_motor;
        driver::DCDriver back_left_motor;

        bool run_pid();
        bool apply_slew_rate();
    };
} // namespace control