#include "drivers/limit.hpp"

namespace driver {

bool DebouncedLimitSwitch::begin(const char *task_name,
                                 uint32_t stack_bytes,
                                 UBaseType_t priority,
                                 BaseType_t core)
{
    if (_task_handle != nullptr) {
        return true;
    }

    pinMode(_pin, _input_mode);
    const bool initially_pressed = _read_pressed();
    _stable_pressed.store(initially_pressed, std::memory_order_release);

    _state_events = xEventGroupCreate();
    if (_state_events == nullptr) {
        return false;
    }

    xEventGroupSetBits(_state_events, initially_pressed ? PRESSED_BIT : RELEASED_BIT);

    const BaseType_t result = xTaskCreatePinnedToCore(
        _task_entry, task_name, stack_bytes, this, priority, &_task_handle, core);

    if (result != pdPASS) {
        _task_handle = nullptr;
        vEventGroupDelete(_state_events);
        _state_events = nullptr;
        return false;
    }

    attachInterruptArg(_pin, _isr_entry, this, CHANGE);

    return true;
}

bool DebouncedLimitSwitch::wait_until_pressed(TickType_t timeout) const
{
    if (_state_events == nullptr) {
        return false;
    }

    const EventBits_t bits =
        xEventGroupWaitBits(_state_events, PRESSED_BIT, pdFALSE, pdTRUE, timeout);
    return (bits & PRESSED_BIT) != 0;
}

bool DebouncedLimitSwitch::wait_until_released(TickType_t timeout) const
{
    if (_state_events == nullptr) {
        return false;
    }

    const EventBits_t bits =
        xEventGroupWaitBits(_state_events, RELEASED_BIT, pdFALSE, pdTRUE, timeout);
    return (bits & RELEASED_BIT) != 0;
}

void DebouncedLimitSwitch::_run()
{
    while (true) {
        // Wait indefinitely for the first GPIO edge.
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        /*
         * Wait until the switch has produced no additional edges for one
         * complete debounce interval.
         *
         * Each new edge restarts the interval.
         */
        while (ulTaskNotifyTake(pdTRUE, _debounce_ticks) > 0) {
        }

        const bool pressed = _read_pressed();
        const bool previous = _stable_pressed.load(std::memory_order_acquire);

        if (pressed == previous) {
            continue;
        }

        _stable_pressed.store(pressed, std::memory_order_release);

        if (pressed) {
            if (_pressed_callback != nullptr) {
                _pressed_callback(_pressed_context);
            }

            xEventGroupClearBits(_state_events, RELEASED_BIT);
            xEventGroupSetBits(_state_events, PRESSED_BIT);
        } else {
            xEventGroupClearBits(_state_events, PRESSED_BIT);
            xEventGroupSetBits(_state_events, RELEASED_BIT);

            if (_released_callback != nullptr) {
                _released_callback(_released_context);
            }
        }
    }
}

} // namespace driver
