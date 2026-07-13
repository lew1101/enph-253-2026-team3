#include "freertos/idf_additions.h"

#include "supervisor.hpp"
#include "esp_log.h"
#include "robot_states.hpp"
#include "control/pid.hpp"
#include "tasks/drive.hpp"

static constexpr char TAG[]{"supervisor"};

using namespace state;

namespace supervisor {
    EventGroupHandle_t g_robot_flags = nullptr;
    namespace {
    RobotJob s_job;
    QueueHandle_t s_job_queue = nullptr;
    QueueHandle_t s_event_queue = nullptr;
    control::PID tape_pid(15.0f, 0.0f, 1.2f);

    void publish_job()
    {
        configASSERT(s_job_queue != nullptr);
        xQueueOverwrite(s_job_queue, &s_job);

        ESP_LOGI(TAG,
                "mode=%s flags=0x%08lx seq=%lu",
                to_string(s_job),
                static_cast<unsigned long>(xEventGroupGetBits(g_robot_flags)));
    }
    } 

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

    // Process the event and update the job accordingly
    while (xQueueReceive(s_event_queue, &event, 0) == pdTRUE) {
        switch (s_job) {
            case RobotJob::ROBOT_IDLE:
                if (event == RobotEvent::START_REQUESTED) {
                    s_job = RobotJob::ROBOT_DRIVE_TO_TARGET;
                }
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

    // If job change, switch up the flags
    if (s_job != prev_job) {
        ESP_LOGD(TAG, "job: %s", to_string(s_job));
        
        // Wipe all flags when changing jobs to avoid stale flags
        xEventGroupClearBits(g_robot_flags, RobotFlag::ROBOT_FLAGS_ALL);

        switch(s_job) {
            case RobotJob::ROBOT_IDLE:
                ESP_LOGI(TAG, "Robot is now idle. All flags cleared.");
                break;
            case RobotJob::ROBOT_CALIBRATE:
                break;
            case RobotJob::ROBOT_FOLLOW_TAPE:
                xEventGroupSetBits(g_robot_flags, RobotFlag::ROBOT_FLAG_DRIVE_ENABLED);
                xEventGroupSetBits(g_robot_flags, RobotFlag::ROBOT_FLAG_TAPE_ACTIVE);
                ESP_LOGI(TAG, "Tape follow job started. Drive and tape sensing enabled.");
                break;
            case RobotJob::ROBOT_DRIVE_TO_TARGET:
                xEventGroupSetBits(g_robot_flags, RobotFlag::ROBOT_FLAG_DRIVE_ENABLED);
                break;
            case RobotJob::ROBOT_FIND_ROCK:
                break;
            case RobotJob::ROBOT_DETECT_METAL:
                xEventGroupSetBits(g_robot_flags, RobotFlag::ROBOT_FLAG_METAL_ENABLED);
                break;
            case RobotJob::ROBOT_ESTOP:
                break;
            case RobotJob::ROBOT_ERROR:
                break;
        }
    }

    switch (s_job) {
        case RobotJob::ROBOT_IDLE:
            break;
        case RobotJob::ROBOT_CALIBRATE:
            break;
        case RobotJob::ROBOT_FOLLOW_TAPE:
            tape_follow();
            break;
        case RobotJob::ROBOT_DRIVE_TO_TARGET:
            break;
        case RobotJob::ROBOT_FIND_ROCK:
            break;
        case RobotJob::ROBOT_DETECT_METAL:
            break;
        case RobotJob::ROBOT_ESTOP:
            break;
        case RobotJob::ROBOT_ERROR:
            break;
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

void tape_follow()
{
    // Update PID controller
        float dt_s = 0.02f; 
        float correction = tape_pid.update(0.0f, g_tape_error.load(), dt_s);

        // Send drive command based on PID correction
        DriveCommand cmd;
        cmd.mode = DriveMode::SET_SPEED;
        cmd.x_speed = 0.0f; // No strafe
        cmd.y_speed = 50.0f; // Constant forward speed
        cmd.rot_speed = correction; // Apply correction to rotation

        send_drive_cmd(cmd);
}
} // namespace supervisor

