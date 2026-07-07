#include <Arduino.h>
#include "sensors/metal_detector.hpp"

static constexpr char TAG[] = "main";

void setup()
{
    vTaskDelete(nullptr); // delete the superloop task
}

void loop() {}
