#include <Arduino.h>
#include <array>

#include "actuators/dc_driver.hpp"

namespace control {

using std::array;

class Drivetrain {
  public:
    struct Config {
        mcpwm_timer_handle_t timer_0 = nullptr;
        mcpwm_timer_handle_t timer_1 = nullptr;

        driver::DCDriver::Config front_right_motor_config;
        driver::DCDriver::Config back_right_motor_config;
        driver::DCDriver::Config front_left_motor_config;
        driver::DCDriver::Config back_left_motor_config;

        float rot_scalar = 0.2015f;
        float acceleration_rate = 0.2f; // percentage per update
    };

    Drivetrain() = default;

    explicit Drivetrain(const Config &config);

    esp_err_t init();

    bool forward(float speed_percentage); // -100 to 100
    bool strafe(float speed_percentage);  // -100 to 100
    bool turn(float speed_percentage);    // -100 to 100, positive is clockwise, negative is
                                          // counter-clockwise

    bool move_vector(float vx, float vy, float omega);
    bool stop();

    bool update();

  private:
    Config _config;
    bool stopped = true;

    int timer_resolution_hz = 1'000'000;
    int PWM_frequency_hz = 200;
    float period_tick = (static_cast<float>(timer_resolution_hz) / (PWM_frequency_hz * 2));

    driver::DCDriver front_right_motor;
    driver::DCDriver front_left_motor;
    driver::DCDriver back_right_motor;
    driver::DCDriver back_left_motor;

    // acceleration parameters
    // front right, back right, front left, back left
    array<float, 4> target_speed{0.0, 0.0, 0.0, 0.0};
    array<float, 4> current_speed{0.0, 0.0, 0.0, 0.0};
    uint32_t previous_time = 0;
};
} // namespace control
