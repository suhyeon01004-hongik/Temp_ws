#include <gtest/gtest.h>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <array>
#include <cstdint>
#include <stdexcept>

#include "vehicle_control/udp_sender.hpp"

namespace vehicle_control {
namespace {

TEST(UdpSender, SendsExactDatagramToLoopbackReceiver) {
  const int receiver = ::socket(AF_INET, SOCK_DGRAM, 0);
  ASSERT_GE(receiver, 0);

  struct sockaddr_in address {};
  address.sin_family = AF_INET;
  address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  address.sin_port = htons(0U);
  ASSERT_EQ(0, ::bind(receiver, reinterpret_cast<struct sockaddr*>(&address),
                      sizeof(address)));

  socklen_t address_size = sizeof(address);
  ASSERT_EQ(0, ::getsockname(
                   receiver, reinterpret_cast<struct sockaddr*>(&address),
                   &address_size));

  struct timeval timeout {};
  timeout.tv_sec = 1;
  ASSERT_EQ(0, ::setsockopt(receiver, SOL_SOCKET, SO_RCVTIMEO, &timeout,
                           sizeof(timeout)));

  const UdpSender sender("127.0.0.1", ntohs(address.sin_port));
  const std::array<std::uint8_t, 3U> expected{{0x10U, 0x20U, 0x30U}};
  sender.send(expected.data(), expected.size());

  std::array<std::uint8_t, 8U> buffer{};
  const ssize_t received =
      ::recv(receiver, buffer.data(), buffer.size(), 0);
  ASSERT_EQ(3, received);
  EXPECT_EQ(expected[0], buffer[0]);
  EXPECT_EQ(expected[1], buffer[1]);
  EXPECT_EQ(expected[2], buffer[2]);

  EXPECT_EQ(0, ::close(receiver));
}

TEST(UdpSender, InvalidIpv4AddressIsRejected) {
  EXPECT_THROW(
      { const UdpSender sender("not-an-ip", 9095U); },
      std::invalid_argument);
}

TEST(UdpSender, ZeroDestinationPortIsRejected) {
  EXPECT_THROW(
      { const UdpSender sender("127.0.0.1", 0U); },
      std::invalid_argument);
}

TEST(UdpSender, NullPayloadIsRejected) {
  const UdpSender sender("127.0.0.1", 9095U);
  EXPECT_THROW(sender.send(nullptr, 1U), std::invalid_argument);
}

}  // namespace
}  // namespace vehicle_control

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
