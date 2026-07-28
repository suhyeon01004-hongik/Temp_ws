#include "vehicle_control/udp_receiver.hpp"

#include <arpa/inet.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <stdexcept>
#include <string>

namespace vehicle_control {
namespace {

std::runtime_error socketError(const std::string& operation) {
  return std::runtime_error(operation + ": " + std::strerror(errno));
}

}  // namespace

UdpReceiver::UdpReceiver(const std::string& listen_ip,
                         std::uint16_t listen_port) {
  struct sockaddr_in address {};
  address.sin_family = AF_INET;
  address.sin_port = htons(listen_port);
  if (::inet_pton(AF_INET, listen_ip.c_str(), &address.sin_addr) != 1) {
    throw std::invalid_argument("UDP listener is not a valid IPv4 address");
  }

  socket_fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
  if (socket_fd_ < 0) {
    throw socketError("failed to create UDP receiver socket");
  }

  const int flags = ::fcntl(socket_fd_, F_GETFL, 0);
  if (flags < 0 || ::fcntl(socket_fd_, F_SETFL, flags | O_NONBLOCK) < 0) {
    const std::runtime_error error =
        socketError("failed to make UDP receiver nonblocking");
    ::close(socket_fd_);
    socket_fd_ = -1;
    throw error;
  }

  if (::bind(socket_fd_, reinterpret_cast<struct sockaddr*>(&address),
             sizeof(address)) < 0) {
    const std::runtime_error error =
        socketError("failed to bind UDP receiver");
    ::close(socket_fd_);
    socket_fd_ = -1;
    throw error;
  }

  socklen_t address_size = sizeof(address);
  if (::getsockname(socket_fd_,
                    reinterpret_cast<struct sockaddr*>(&address),
                    &address_size) < 0) {
    const std::runtime_error error =
        socketError("failed to read UDP receiver port");
    ::close(socket_fd_);
    socket_fd_ = -1;
    throw error;
  }
  bound_port_ = ntohs(address.sin_port);
}

UdpReceiver::~UdpReceiver() {
  if (socket_fd_ >= 0) {
    ::close(socket_fd_);
  }
}

std::size_t UdpReceiver::receive(std::uint8_t* data,
                                 std::size_t capacity) const {
  if (data == nullptr) {
    throw std::invalid_argument("UDP receive buffer is null");
  }
  if (capacity == 0U) {
    throw std::invalid_argument("UDP receive buffer capacity must be positive");
  }

  ssize_t received = -1;
  do {
    received = ::recv(socket_fd_, data, capacity, MSG_TRUNC);
  } while (received < 0 && errno == EINTR);

  if (received < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
    return 0U;
  }
  if (received < 0) {
    throw socketError("failed to receive UDP datagram");
  }
  if (static_cast<std::size_t>(received) > capacity) {
    throw std::runtime_error("UDP datagram exceeds receive buffer");
  }
  return static_cast<std::size_t>(received);
}

std::uint16_t UdpReceiver::boundPort() const { return bound_port_; }

}  // namespace vehicle_control
