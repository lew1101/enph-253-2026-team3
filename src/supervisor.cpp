#include "freertos/idf_additions.h"

#include "supervisor.hpp"
#include "esp_log.h"

static constexpr char TAG[]{"supervisor"};

EventGroupHandle_t g_flags = nullptr;

namespace supervisor {
namespace {
RobotState s_state;
QueueHandle_t s_state_queue = nullptr;
QueueHandle_t s_event_queue = nullptr;

void publish_state()
{
    configASSERT(s_state_queue == nullptr);
    xQueueOverwrite(s_state_queue, &s_state);

    ESP_LOGI(TAG,
             "mode=%s flags=0x%08lx seq=%lu",
             to_string(s_state),
             static_cast<unsigned long>(xEventGroupGetBits(g_robot_flags)));
}
} // namespace

void init(RobotState &initial_state)
{
    s_event_queue = xQueueCreate(EVENT_QUEUE_LEN, sizeof(RobotState));
    configASSERT(s_event_queue != nullptr);

    s_state_queue = xQueueCreate(1, sizeof(RobotState));
    configASSERT(s_state_queue != nullptr);

    s_state = initial_state;

    publish_state();
}

void update()
{
    configASSERT(s_event_queue != nullptr);
    configASSERT(s_state_queue != nullptr);
    configASSERT(g_robot_flags != nullptr);

    EventBits_t flags = xEventGroupGetBits(g_robot_flags);
    RobotEvent event;
    RobotState prev_state = s_state;

    while (xQueueReceive(s_event_queue, &event, 0) == pdTRUE) {
        switch (s_state) {
            case RobotState::ROBOT_IDLE:
                break;
            case RobotState::ROBOT_CALIBRATE:
            case RobotState::ROBOT_FOLLOW_TAPE:
            case RobotState::ROBOT_FIND_ROCK:
            case RobotState::ROBOT_DETECT_METAL:
            case RobotState::ROBOT_ESTOP:
            case RobotState::ROBOT_ERROR:
                break;
            case RobotState::ROBOT_DRIVE_TO_TARGET:
                break;
        }
    }

    if (s_state != prev_state) {
        ESP_LOGD(TAG, "state: %s", to_string(s_state));
    }

    publish_state();
}

bool send_event(RobotEvent event, TickType_t timeout)
{
    configASSERT(s_event_queue != nullptr);
    return xQueueSend(s_event_queue, &event, timeout) == pdTRUE;
}

bool send_event_from_isr(RobotEvent event, BaseType_t *higher_priority_task_woken)
{
    configASSERT(s_event_queue != nullptr);
    return xQueueSendFromISR(s_event_queue, &event, higher_priority_task_woken) == pdTRUE;
}

bool get_state(RobotState &out, TickType_t timeout)
{
    configASSERT(s_state_queue != nullptr);
    return xQueuePeek(s_state_queue, &out, timeout) == pdTRUE;
}
} // namespace supervisor
