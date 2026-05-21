#pragma once

#include "comms/tlv_parser.hpp"
#include "comms/tlv_fields.hpp"

#include <Arduino.h>

namespace telemetry {
struct RobotTelemetry {
    float battery_v = 0.0f;
    float drivetrain_l = 0.0f;
    float drivetrain_r = 0.0f;
    int32_t err = 0;

    uint32_t tick = 0;
};

struct RobotCommand {
    float drivetrain_l = 0.0f;
    float drivetrain_r = 0.0f;
    uint8_t estop = 0ul;

    uint16_t sequence = 0;
    uint32_t tick = 0;
    bool valid = false;
};

enum PacketType : uint8_t {
    TELEMETRY = 1,
    COMMAND = 2,
    ACK = 3, // not implemented
    PING = 4,
    PONG = 5
    // Add more packet types as needed
};

enum TlvType : uint8_t {
    BATT_V = 1,
    DRIVETRAIN_L = 2,
    DRIVETRAIN_R = 3,
    LOG = 4,
    ERR = 5,
    // Add more TLV types as needed

    CMD_DRIVETRAIN_L = 17,
    CMD_DRIVETRAIN_R = 18,
    CMD_ESTOP = 19
};

struct RobotCallbacks {
    using ReadTelemetryCallback = esp_err_t (*)(RobotTelemetry &telemetry, void *state);
    using HandleCommandCallback = esp_err_t (*)(const RobotCommand &command, void *state);

    ReadTelemetryCallback read_telemetry = nullptr;
    HandleCommandCallback handle_command = nullptr;
    void *param = nullptr;
};

enum class PacketError : uint8_t {
    None,
    ParseFail,
    PacketTooLarge,
    InvalidPacket,
    InternalError,
};

enum class ResponseType : uint8_t {
    None,
    Reply,
};

struct PacketHandlerResult {
    PacketError error = PacketError::None;
    ResponseType response_type = ResponseType::None;
    size_t tx_len = 0;

    inline bool ok() const { return error == PacketError::None; }
    inline bool should_send() const { return ok() && response_type == ResponseType::Reply && tx_len > 0; }
};

class PacketHandler {
  private:
    uint8_t *_tx_buf;
    const uint8_t *_rx_buf;

    const size_t _tx_cap;
    const size_t _rx_cap;

    const RobotCallbacks &_robot_cbs;

  public:
    PacketHandler(const RobotCallbacks &robot_cbs,
                  uint8_t *tx_buf,
                  size_t tx_cap,
                  const uint8_t *rx_buf,
                  size_t rx_cap)
        : _tx_buf{tx_buf}
        , _rx_buf{rx_buf}
        , _tx_cap{tx_cap}
        , _rx_cap{rx_cap}
        , _robot_cbs{robot_cbs}
    {
    }

    PacketHandlerResult handle_received_packet(size_t rx_len);
    PacketHandlerResult build_telemetry_packet(uint16_t &sequence);
    PacketHandlerResult build_pong_packet(uint16_t ping_seq);

  private:
    PacketHandlerResult handle_command(TlvPacketParser &parser);
    PacketHandlerResult handle_ping(TlvPacketParser &parser, uint16_t sequence);
};
} // namespace telemetry
