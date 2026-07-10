#include "freertos/idf_additions.h"

#include "supervisor.hpp"
#include "esp_log.h"

static constexpr char TAG[]{"supervisor"};

EventGroupHandle_t g_flags = nullptr;

namespace supervisor {
namespace {
RobotJob s_job;
QueueHandle_t s_job_queue = nullptr;
QueueHandle_t s_event_queue = nullptr;

void publish_job()
{
    configASSERT(s_job_queue == nullptr);
    xQueueOverwrite(s_job_queue, &s_job);

    ESP_LOGI(TAG,
             "mode=%s flags=0x%08lx seq=%lu",
             to_string(s_job),
             static_cast<unsigned long>(xEventGroupGetBits(g_robot_flags)));
}
} // namespace

void init(RobotJob &initial_job)
{
    s_event_queue = xQueueCreate(EVENT_QUEUE_LEN, sizeof(RobotJob));
    configASSERT(s_event_queue != nullptr);

    s_job_queue = xQueueCreate(1, sizeof(RobotJob));
    configASSERT(s_job_queue != nullptr);

    s_job = initial_job;

    publish_job();
}

void update()
{
    configASSERT(s_event_queue != nullptr);
    configASSERT(s_job_queue != nullptr);
    configASSERT(g_robot_flags != nullptr);

    EventBits_t flags = xEventGroupGetBits(g_robot_flags);
    RobotEvent event;
    RobotJob prev_job = s_job;

    while (xQueueReceive(s_event_queue, &event, 0) == pdTRUE) {
        switch (s_job) {
            case RobotJob::ROBOT_IDLE:
                break;
            case RobotJob::ROBOT_CALIBRATE:
            case RobotJob::ROBOT_FOLLOW_TAPE:
            case RobotJob::ROBOT_FIND_ROCK:
            case RobotJob::ROBOT_DETECT_METAL:
            case RobotJob::ROBOT_ESTOP:
            case RobotJob::ROBOT_ERROR:
                break;
            case RobotJob::ROBOT_DRIVE_TO_TARGET:
                break;
        }
    }

    if (s_job != prev_job) {
        ESP_LOGD(TAG, "job: %s", to_string(s_job));
    }

    publish_job();
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

bool get_job(RobotJob &out, TickType_t timeout)
{
    configASSERT(s_job_queue != nullptr);
    return xQueuePeek(s_job_queue, &out, timeout) == pdTRUE;
}
} // namespace supervisor

