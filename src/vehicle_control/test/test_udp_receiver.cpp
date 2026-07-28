#include <gtest/gtest.h>

#include <unistd.h>

#include <array>
#include <cstdint>
#include <stdexcept>

#include "vehicle_control/udp_receiver.hpp"
#include "vehicle_control/udp_sender.hpp"

namespace vehicle_control {
namespace {

TEST(UdpReceiver, ReceivesExactLoopbackDatagram) {
  UdpReceiver receiver("127.0.0.1", 0U);
  const UdpSender sender("127.0.0.1", receiver.boundPort());
  const std::array<std::uint8_t, 3U> expected{{0x10U, 0x20U, 0x30U}};
  sender.send(expected.data(), expected.size());

  std::array<std::uint8_t, 8U> actual{};
  std::size_t received = 0U;
  for (int attempt = 0; attempt < 100 && received == 0U; ++attempt) {
    received = receiver.receive(actual.data(), actual.size());
    if (received == 0U) {
      ::usleep(1000U);
    }
  }

  ASSERT_EQ(expected.size(), received);
  EXPECT_EQ(expected[0], actual[0]);
  EXPECT_EQ(expected[1], actual[1]);
  EXPECT_EQ(expected[2], actual[2]);
}

TEST(UdpReceiver, EmptyNonblockingReceiveReturnsZero) {
  UdpReceiver receiver("127.0.0.1", 0U);
  std::array<std::uint8_t, 8U> buffer{};

  EXPECT_EQ(0U, receiver.receive(buffer.data(), buffer.size()));
}

TEST(UdpReceiver, InvalidIpv4AddressIsRejected) {
  EXPECT_THROW(
      { const UdpReceiver receiver("not-an-ip", 9094U); },
      std::invalid_argument);
}

TEST(UdpReceiver, NullBufferIsRejected) {
  UdpReceiver receiver("127.0.0.1", 0U);

  EXPECT_THROW(receiver.receive(nullptr, 1U), std::invalid_argument);
}

TEST(UdpReceiver, ZeroCapacityIsRejected) {
  UdpReceiver receiver("127.0.0.1", 0U);
  std::array<std::uint8_t, 1U> buffer{};

  EXPECT_THROW(receiver.receive(buffer.data(), 0U), std::invalid_argument);
}

}  // namespace
}  // namespace vehicle_control

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
