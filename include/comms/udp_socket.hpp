#pragma once

#include "esp_err.h"

#include "lwip/sockets.h"

#include <cstdint>

namespace telemetry {
class UdpSocket {
  public:
    UdpSocket() = default;
    ~UdpSocket();

    // delete copy constructors
    UdpSocket(const UdpSocket &) = delete;
    UdpSocket &operator=(const UdpSocket &) = delete;

    esp_err_t open(const char *dest_ip, uint16_t port);
    esp_err_t close();

    inline bool is_open() const { return _sock >= 0; }

    esp_err_t send(const uint8_t *data, size_t len) const;
    esp_err_t receive(uint8_t *buffer, size_t capacity, size_t &out_len) const;

  private:
    int _sock = -1;
    sockaddr_in _remote_addr{};
};

} // namespace telemetry
