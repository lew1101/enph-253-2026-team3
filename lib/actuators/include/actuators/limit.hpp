#pragma once

#include "esp32-hal-gpio.h"
#include "projdefs.h"

#include <atomic>
#include <cstdint>

class DebouncedLimitSwitch {
    using Callback = void (*)(void *context);

  public:
    DebouncedLimitSwitch(gpio_num_t pin,
                         int input_mode = INPUT,
                         int active_level = LOW,
                         uint32_t debounce_ms = 20)
        : _pin(pin)
        , _input_mode(input_mode)
        , _active_level(active_level)
        , _debounce_ticks(pdMS_TO_TICKS(debounce_ms))
    {
    }

    bool begin(const char *task_name = "limit_switch",
               uint32_t stack_bytes = 2048,
               UBaseType_t priority = 4,
               BaseType_t core = tskNO_AFFINITY);

    inline void on_pressed(Callback callback, void *context = nullptr)
    {
        _pressed_callback = callback;
        _pressed_context = context;
    }

    inline void on_released(Callback callback, void *context = nullptr)
    {
        _released_callback = callback;
        _released_context = context;
    }

    [[nodiscard]] inline bool is_pressed() const
    {
        return _stable_pressed.load(std::memory_order_acquire);
    }

  private:
    [[nodiscard]] inline bool _read_pressed() const { return digitalRead(_pin) == _active_level; }

    static void ARDUINO_ISR_ATTR _isr_entry(void *argument)
    {
        auto *self = static_cast<DebouncedLimitSwitch *>(argument);
        BaseType_t higher_priority_task_woken = pdFALSE;
        vTaskNotifyGiveFromISR(self->_task_handle, &higher_priority_task_woken);
        portYIELD_FROM_ISR(higher_priority_task_woken);
    }

    static void _task_entry(void *argument)
    {
        static_cast<DebouncedLimitSwitch *>(argument)->_run();
    }

    void _run();

    const uint8_t _pin;
    const uint8_t _active_level;
    const uint8_t _input_mode;
    const TickType_t _debounce_ticks;

    TaskHandle_t _task_handle = nullptr;
    std::atomic_bool _stable_pressed{false};

    Callback _pressed_callback = nullptr;
    Callback _released_callback = nullptr;

    void *_pressed_context = nullptr;
    void *_released_context = nullptr;
};
