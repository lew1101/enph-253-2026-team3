#include "drive_command_handler.hpp"
#include "FreeRTOS.h"
#include "portmacro.h"

DriveCommandHandler::DriveCommandHandler(TickType_t retry_period, uint32_t max_retries)
    : _retry_period{retry_period}
    , _max_retries{max_retries}
{
    configASSERT(_retry_period > 0);

    _mutex = xSemaphoreCreateMutex();
    configASSERT(_mutex != nullptr);
}

DriveCommandHandler::~DriveCommandHandler()
{
    if (_mutex != nullptr) {
        vSemaphoreDelete(_mutex);
        _mutex = nullptr;
    }
}

void DriveCommandHandler::submit(const robot_RobotUartMessage &message, TickType_t now)
{
    xSemaphoreTake(_mutex, portMAX_DELAY);
    _message = message;
    _last_send = now;
    _retry_count = 0;
    _failed = false;
    _pending = true;
    xSemaphoreGive(_mutex);
}

void DriveCommandHandler::acknowledge(uint32_t sequence)
{
    xSemaphoreTake(_mutex, portMAX_DELAY);
    _ack = sequence;
    xSemaphoreGive(_mutex);
}

bool DriveCommandHandler::retry_due(TickType_t now, robot_RobotUartMessage *out)
{
    configASSERT(out != nullptr);
    xSemaphoreTake(_mutex, portMAX_DELAY);

    bool is_retry_due = false;

    if (!_pending || now - _last_send < _retry_period) {
        is_retry_due = false;
    } else if (_ack == _message.payload.drive_command.sequence) {
        _pending = false;

        is_retry_due = false;
    } else if (_retry_count >= _max_retries) {
        _pending = false;
        _failed = true;

        is_retry_due = false;
    } else {
        ++_retry_count;
        _last_send = now;
        *out = _message;

        is_retry_due = true;
    }
    xSemaphoreGive(_mutex);

    return is_retry_due;
}

bool DriveCommandHandler::pending() const
{
    xSemaphoreTake(_mutex, portMAX_DELAY);
    bool pending = _pending;
    xSemaphoreGive(_mutex);
    return pending;
}

bool DriveCommandHandler::failed() const
{
    xSemaphoreTake(_mutex, portMAX_DELAY);
    bool failed = _failed;
    xSemaphoreGive(_mutex);
    return failed;
}
