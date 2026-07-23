#pragma once

#include <atomic>

#include "freertos/idf_additions.h"

#include "drive_message.pb.h"
#include "robot_message.pb.h"
#include "esp_err.h"

class DriveMessageHandler {
  public:
    DriveMessageHandler();
    DriveMessageHandler(const DriveMessageHandler &) = delete;
    DriveMessageHandler &operator=(const DriveMessageHandler &) = delete;

    esp_err_t handle(const robot_RobotUartMessage &message);
    void on_link_disconnected();
    drive_DriveUartMessage make_status(bool link_connected, uint32_t uptime_ms) const;

  private:
    esp_err_t _apply_command(robot_DriveCommand message);
    void _stop();
    static bool _is_newer_sequence(uint32_t sequence, uint32_t previous);

    SemaphoreHandle_t _mutex = nullptr;

    uint32_t _last_sequence{0};
    uint32_t _session_id{0};
    uint32_t _fault{0};

    bool _have_sequence{false};
    bool _have_session{false};
    bool _estop{false};
    bool _drive_enabled{false};
    bool _tape_enabled{false};
    bool _tape_command_active{false};
};
