#include <Arduino.h>
#include "actuators/dc_driver.hpp"

class Drivetrain {
    public:
        struct Config{
            
        };

        explicit Drivetrain(const Config &config); 

        bool init();
        
        bool forward(float meter);
        bool left (float meter);
        bool right (float meter);

        bool turn (float degree);
        
    };