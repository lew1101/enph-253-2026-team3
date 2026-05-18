#include "comms/udp_socket.hpp"

#include "esp_err.h"
#include "esp_log.h"

#include <cerrno>

static constexpr const char TAG[] = "udp_socket";

namespace telemetry {
UdpSocket::~UdpSocket() { close(); }

esp_err_t UdpSocket::open(const char *dest_ip, uint16_t port)
{
    close(); // close before opening new socket

    // use IPv4, datagrams, udp
    // Create an IPv4 UDP socket.
    _sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (_sock < 0) {
        // socket creation failed
        ESP_LOGE(TAG, "socket() failed, errno=%d", errno);
        return ESP_FAIL;
    }

    // enable broadcast
    int enable = 1;
    int result = setsockopt(_sock, SOL_SOCKET, SO_BROADCAST, &enable, sizeof(enable));
    if (result < 0) {
        // enable broadcast failed
        ESP_LOGE(TAG, "setsockopt(SO_BROADCAST) failed, errno=%d", errno);
        close();
        return ESP_FAIL;
    }

    sockaddr_in local_addr{};
    local_addr.sin_family = AF_INET; // destination ip in IPv4

    // bind this UDP socket to this port on all local IP addresses.
    // addresses are uint32
    local_addr.sin_addr.s_addr = htonl(INADDR_ANY); // htonl = host to network long

    // set the local UDP port
    // ports are uint16
    local_addr.sin_port = htons(port); // htons = host to network short

    // bind socket to local port
    result = bind(_sock, reinterpret_cast<const sockaddr *>(&local_addr), sizeof(local_addr));
    if (result < 0) {
        // bind failed
        ESP_LOGE(TAG, "bind() failed, errno=%d", errno);
        close();
        return ESP_FAIL;
    }

    _remote_addr = {};
    _remote_addr.sin_family = AF_INET;   // remote ip in IPv4
    _remote_addr.sin_port = htons(port); // set the remote UDP port

    // converts the destination IP string into binary socket format.
    result = inet_pton(AF_INET, dest_ip, &_remote_addr.sin_addr);
    if (result != 1) {
        // invalid ip string
        ESP_LOGE(TAG, "Invalid destination IP: %s", dest_ip);
        close();
        return ESP_ERR_INVALID_ARG;
    }

    // get current socket flags
    int flags = fcntl(_sock, F_GETFL, 0);
    if (flags < 0) {
        ESP_LOGE(TAG, "fcntl(F_GETFL) failed, errno=%d", errno);
        close();
        return ESP_FAIL;
    }

    // make recvfrom nonblocking
    result = fcntl(_sock, F_SETFL, flags | O_NONBLOCK);
    if (result < 0) {
        ESP_LOGE(TAG, "fcntl(F_SETFL) failed, errno=%d", errno);
        close();
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "UDP socket open: dest=%s:%u local_port=%u", dest_ip, port, port);

    return ESP_OK;
}

esp_err_t UdpSocket::close()
{
    if (_sock < 0) return ESP_OK; // socket never opened.

    int old_sock = _sock;
    _sock = -1;

    if (::close(old_sock) < 0) {
        ESP_LOGW(TAG, "close() failed, errno=%d", errno);
        return ESP_FAIL;
    }

    return ESP_OK;
}

esp_err_t UdpSocket::send(const uint8_t *data, size_t len) const
{
    if (_sock < 0) return ESP_ERR_INVALID_STATE;
    if (data == nullptr || len == 0) return ESP_ERR_INVALID_ARG;

    int sent = sendto(_sock,
                      data,
                      len,
                      0,
                      reinterpret_cast<const sockaddr *>(&_remote_addr),
                      sizeof(_remote_addr));

    if (sent < 0) {
        ESP_LOGW(TAG, "sendto() failed, errno=%d", errno);
        return ESP_FAIL;
    }

    if (static_cast<size_t>(sent) != len) {
        ESP_LOGW(TAG, "sendto() sent partial packet");
        return ESP_FAIL;
    }

    return ESP_OK;
}

esp_err_t UdpSocket::receive(uint8_t *buffer, size_t capacity, size_t &out_len) const
{
    out_len = 0;

    if (_sock < 0) return ESP_ERR_INVALID_STATE;
    if (buffer == nullptr || capacity == 0) return ESP_ERR_INVALID_ARG;

    sockaddr_in source_addr{};
    socklen_t source_addr_len = sizeof(source_addr);

    int received = recvfrom(
        _sock, buffer, capacity, 0, reinterpret_cast<sockaddr *>(&source_addr), &source_addr_len);

    if (received < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK)
            return ESP_ERR_NOT_FOUND; // no packet found, ok.

        // recvfrom actually failed.
        ESP_LOGW(TAG, "recvfrom() failed, errno=%d", errno);
        return ESP_FAIL;
    }

    out_len = static_cast<size_t>(received);
    return ESP_OK;
}
} // namespace telemetry
