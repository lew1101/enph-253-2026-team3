#include "freertos/idf_additions.h"
#include "tasks/drive.hpp"
#include "tasks/tape_sense.hpp"
#include "tasks/imu.hpp"
#include "tasks/uart.hpp"

void setup() {


    vTaskDelete(nullptr);
}

void loop() {}
