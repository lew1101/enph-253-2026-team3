#include "actuators/limit.hpp"

bool DebouncedLimitSwitch::begin(const char *task_name ,
                                 uint32_t stack_bytes,
                                 UBaseType_t priority,
                                 BaseType_t core)
{
    if (_task_handle != nullptr) {
        return true;
    }

    pinMode(_pin, _input_mode);
    _stable_pressed.store(_read_pressed(), std::memory_order_release);

    const BaseType_t result = xTaskCreatePinnedToCore(
        _task_entry, task_name, stack_bytes, this, priority, &_task_handle, core);

    if (result != pdPASS) {
        _task_handle = nullptr;
        return false;
    }

    attachInterruptArg(_pin, _isr_entry, this, CHANGE);

    return true;
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
        } else {
            if (_released_callback != nullptr) {
                _released_callback(_released_context);
            }
        }
    }
}
