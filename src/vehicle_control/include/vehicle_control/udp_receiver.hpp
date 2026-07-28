#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

namespace vehicle_control {

class UdpReceiver {
 public:
  UdpReceiver(const std::string& listen_ip, std::uint16_t listen_port);
  ~UdpReceiver();

  UdpReceiver(const UdpReceiver&) = delete;
  UdpReceiver& operator=(const UdpReceiver&) = delete;

  std::size_t receive(std::uint8_t* data, std::size_t capacity) const;
  std::uint16_t boundPort() const;

 private:
  int socket_fd_{-1};
  std::uint16_t bound_port_{0U};
};

}  // namespace vehicle_control
