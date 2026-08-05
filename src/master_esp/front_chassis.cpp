#include "actuators/elevator_claw.hpp"
#include "actuators/worm_spear.hpp"

#include "front_chassis.hpp"
#include "esp_err.h"
#include "esp_check.h"

static constexpr char TAG[] = "front_chassis";

FastAccelStepperEngine stepper_engine = FastAccelStepperEngine();

control::ElevatorClaw elevator_claw{elevator_config};
control::WormSpear worm_spear{worm_config};

esp_err_t front_chassis_init()
{
    stepper_engine.init();

    const esp_err_t worm_err = worm_spear.init();
    ESP_RETURN_ON_ERROR(worm_err, TAG, "Failed to initialize WormSpear");

    const esp_err_t elevator_err = elevator_claw.init();
    if (elevator_err != ESP_OK) {
        worm_spear.disable();
        ESP_RETURN_ON_ERROR(elevator_err, TAG, "Failed to initialize ElevatorClaw");
    }

    ESP_LOGI(TAG, "WormSpear and ElevatorClaw initialized successfully");
    return ESP_OK;
}
