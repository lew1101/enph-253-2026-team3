#include "freertos/idf_additions.h"

#include "supervisor.hpp"
#include "state.hpp"

#include "esp_log.h"

static constexpr char TAG[]{"supervisor"};

namespace supervisor {
namespace {
SupervisorConfig s_cfg;
void (*s_event_handler)(const RobotEvent &) = nullptr;

RobotState s_robot_state{};
QueueHandle_t s_event_queue = nullptr;
QueueHandle_t s_state_queue = nullptr;

void publish_state()
{
    configASSERT(s_state_queue == nullptr);
    s_robot_state.sequence++;
    xQueueOverwrite(s_state_queue, &s_robot_state);

    ESP_LOGI(TAG,
             "mode=%s flags=0x%08lx seq=%lu",
             to_string(s_robot_state.mode),
             static_cast<unsigned long>(s_robot_state.flags),
             static_cast<unsigned long>(s_robot_state.sequence));
}
} // namespace

void init(SupervisorConfig &cfg, void (*event_handler)(const RobotEvent &))
{
    s_cfg = cfg;
    s_event_queue = xQueueCreate(s_cfg.event_queue_len, sizeof(RobotEvent));
    configASSERT(s_event_queue != nullptr);

    s_state_queue = xQueueCreate(1, sizeof(RobotState));
    configASSERT(s_state_queue != nullptr);

    s_event_handler = event_handler;

    publish_state();
}

void update()
{
    configASSERT(s_event_queue != nullptr);
    RobotEvent event;

    while (xQueueReceive(s_event_queue, &event, 0) == pdTRUE) {
        s_event_handler(event);
    }

    publish_state();
}

bool post_event(RobotEventType type, TickType_t timeout)
{
    configASSERT(s_event_queue != nullptr);
    RobotEvent event{.event_type = type};

    return xQueueSend(s_event_queue, &event, timeout) == pdTRUE;
}

bool post_event_from_isr(RobotEventType type, BaseType_t *higher_priority_task_woken)
{
    configASSERT(s_event_queue != nullptr);
    RobotEvent event{.event_type = type};

    return xQueueSendFromISR(s_event_queue, &event, higher_priority_task_woken) == pdTRUE;
}

bool peek_state(RobotState &out, TickType_t timeout)
{
    configASSERT(s_state_queue != nullptr);
    return xQueuePeek(s_state_queue, &out, timeout) == pdTRUE;
}
} // namespace supervisor
