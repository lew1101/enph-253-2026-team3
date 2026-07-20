#pragma once

#include <atomic>
#include "freertos/FreeRTOS.h"
#include "freertos/idf_additions.h"
#include "portmacro.h"
#include "robot_message.pb.h"

class DriveCommandHandler {
  public:
    DriveCommandHandler(TickType_t retry_period, uint32_t max_retries);
    ~DriveCommandHandler();

    DriveCommandHandler(const DriveCommandHandler &) = delete;
    DriveCommandHandler &operator=(const DriveCommandHandler &) = delete;

    void submit(const robot_RobotUartMessage &message, TickType_t now);
    void acknowledge(uint32_t sequence);
    bool retry_due(TickType_t now, robot_RobotUartMessage *out);
    bool pending() const;
    bool failed() const;

  private:
    SemaphoreHandle_t _mutex;
    robot_RobotUartMessage _message = robot_RobotUartMessage_init_zero;

    TickType_t _last_send = 0;
    TickType_t _retry_period;
    uint32_t _retry_count = 0;
    uint32_t _max_retries;

    uint32_t _ack{0};
    bool _pending{false};
    bool _failed{false};
};
