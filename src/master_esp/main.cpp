#include "freertos/idf_additions.h"

#include "esp_err.h"
#include "esp_log.h"

#include "actuators/limit.hpp"

#include "supervisor.hpp"

#include "tasks/uart.hpp"
#include "tasks/camera_uart.hpp"
#include "tasks/metal.hpp"

#include "front_chassis.hpp"

DebouncedLimitSwitch elev_limit{GPIO_NUM_8};
DebouncedLimitSwitch worm_limit{GPIO_NUM_10};

void setup()
{
    Serial.begin(115200);
    delay(1000);

    supervisor::init();
    supervisor::attach_main_loop();

    ESP_ERROR_CHECK(start_master_uart_tasks());

    ESP_ERROR_CHECK_WITHOUT_ABORT(front_chassis_init());
    ESP_ERROR_CHECK_WITHOUT_ABORT(start_metal_detector_task());

    const esp_err_t camera_uart_err = start_camera_uart_task();
    log_i("camera UART task start: %s", esp_err_to_name(camera_uart_err));
    delay(1000);

    worm_spear.calibrate();
    elevator_claw.calibrate();
}

enum ElevatorPos : int32_t {
    ELEV_FLOOR = 0,
    ELEV_TOWER_1 = 1,
    ELEV_TOWER_2 = 2,
    ELEV_TOWER_3 = 3,
    ELEV_BACK = 4,
};

enum SpearPos : int32_t { SPEAR_LEFT = 50'000, SPEAR_CENTRE = 25'000, SPEAR_RIGHT = 0 };

constexpr float SPEAR_UP = 90.0f;
constexpr float SPEAR_DOWN = 45.0f;

void assemble_tower()
{
    // ===============
    // Build the tower
    /*
    -> Forward -> Spear up -> Backward and center worm ->
    Elevator down -> Claw close -> Elevator up -> Spear down -> Forward
    -> Spear up -> Backward and center worm -> Elevator down -> Claw close -> Elevator up ->
    Spear down
    -> Worm to third tower piece -> Forward
    -> Spear up -> Backward and center worm -> Elevator down -> Claw close -> Elevator up ->
    Spear down
    -> Worm center cresent moon while driving backward -> rotate CW until find tape -> Tape
    follow
    -> Boss tape detected -> forward and back until tape aligned with side sensors -> rotate CCW
    until side tape aligned with front sensors -> Drive forward slowly (cresent moon will auto
    align the robot with the boss) -> Elevator down -> Claw open */

    // move forward
    worm_spear.spear_angle(SPEAR_UP);
    delay(200);

    // move backward
}

void loop()
{
    // PHASE 1: ROCKS and TELETUBBY

    // PHASE 2: TOWER
    //===============
    // Reorient to the tower:
    /* Tower tape detected -> forward and back until tape aligned with side sensors -> rotate CCW
    until side tape aligned with front sensors */

    assemble_tower();

    // PHASE 3: SOLAR PANEL

    vTaskDelay(portMAX_DELAY);
}


