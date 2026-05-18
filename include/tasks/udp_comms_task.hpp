#pragma once

#include <esp_err.h>

#include <cstdint>

namespace telemetry {
static constexpr char UDP_BROADCAST_IP[] = "255.255.255.255";
static constexpr uint16_t UDP_PORT = 5005;
static constexpr uint32_t PERIOD_MS = 50;
static constexpr uint32_t TASK_STACK_SIZE = 4096;
static constexpr uint32_t TASK_PRIORITY = 5;

struct RobotTelemetry {
    float battery_v = 0.0f;
    float drivetrain_l = 0.0f;
    float drivetrain_r = 0.0f;
    int32_t err = 0;

    uint16_t sequence = 0;
    uint32_t tick = 0;
};

struct RobotCommand {
    float drivetrain_l = 0.0f;
    float drivetrain_r = 0.0f;
    bool estop = false;

    uint16_t sequence = 0;
    uint32_t tick = 0;
    bool valid = false;
};

enum PacketType : uint8_t {
    TELEMETRY = 1,
    COMMAND = 2,
    ACK = 3,
    // Add more packet types as needed
};

enum TlvType : uint8_t {
    BATT_V = 1,
    DRIVETRAIN_L = 2,
    DRIVETRAIN_R = 3,
    ERR = 4,
    // Add more TLV types as needed

    CMD_DRIVETRAIN_L = 17,
    CMD_DRIVETRAIN_R = 18,
    CMD_ESTOP = 19
};

struct UdpCommsConfig {
    const char *dest_ip;
    uint16_t port;
    uint32_t period_ms;
    uint32_t stack_size;
    uint32_t priority;
};

struct UdpCommsCallbacks {
    using ReadTelemetryCallback = esp_err_t (*)(RobotTelemetry &telemetry, void *state);
    using HandleCommandCallback = esp_err_t (*)(const RobotCommand &command, void *state);

    ReadTelemetryCallback read_telemetry = nullptr;
    HandleCommandCallback handle_command = nullptr;
    void *state = nullptr;
};

static constexpr UdpCommsConfig UDP_COMM_CONFIG{.dest_ip = UDP_BROADCAST_IP,
                                                .port = UDP_PORT,
                                                .period_ms = PERIOD_MS,
                                                .stack_size = TASK_STACK_SIZE,
                                                .priority = TASK_PRIORITY};

esp_err_t start_udp_comms_task(const UdpCommsConfig &config, const UdpCommsCallbacks &callbacks);

} // namespace telemetry
