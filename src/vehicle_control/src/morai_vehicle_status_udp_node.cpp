#include <ros/ros.h>

#include <array>
#include <cmath>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>

#include "vehicle_control/VehicleStatus.h"
#include "vehicle_control/morai_vehicle_status_packet.hpp"
#include "vehicle_control/udp_receiver.hpp"

namespace vehicle_control {
namespace {

class MoraiVehicleStatusUdpNode {
 public:
  MoraiVehicleStatusUdpNode() : private_node_("~") {
    std::string listen_ip;
    std::string status_topic;
    int listen_port = 9094;
    double poll_rate = 200.0;
    private_node_.param<std::string>("listen_ip", listen_ip, "0.0.0.0");
    private_node_.param("listen_port", listen_port, 9094);
    private_node_.param<std::string>(
        "status_topic", status_topic, "/vehicle/status");
    private_node_.param("poll_rate", poll_rate, 200.0);

    if (listen_port < 1 || listen_port > 65535) {
      throw std::invalid_argument("listen_port must be in 1..65535");
    }
    if (!std::isfinite(poll_rate) || poll_rate <= 0.0) {
      throw std::invalid_argument("poll_rate must be positive");
    }

    receiver_.reset(new UdpReceiver(
        listen_ip, static_cast<std::uint16_t>(listen_port)));
    status_publisher_ =
        node_.advertise<vehicle_control::VehicleStatus>(status_topic, 10);
    poll_timer_ = node_.createWallTimer(
        ros::WallDuration(1.0 / poll_rate),
        &MoraiVehicleStatusUdpNode::onPoll, this);

    ROS_INFO("MORAI vehicle status UDP: %s:%d -> %s",
             listen_ip.c_str(), listen_port, status_topic.c_str());
  }

 private:
  void onPoll(const ros::WallTimerEvent&) {
    constexpr std::size_t kMaximumPacketsPerPoll = 64U;
    for (std::size_t count = 0U; count < kMaximumPacketsPerPoll; ++count) {
      std::size_t received = 0U;
      try {
        received = receiver_->receive(buffer_.data(), buffer_.size());
      } catch (const std::exception& error) {
        ROS_ERROR_THROTTLE(1.0, "MORAI status UDP receive failed: %s",
                           error.what());
        return;
      }
      if (received == 0U) {
        return;
      }

      MoraiVehicleStatus status;
      std::string error;
      if (!decodeMoraiVehicleStatus(
              buffer_.data(), received, &status, &error)) {
        ROS_WARN_THROTTLE(1.0, "invalid MORAI status packet: %s",
                          error.c_str());
        continue;
      }

      vehicle_control::VehicleStatus message;
      message.header.stamp = ros::Time::now();
      message.control_mode = status.control_mode;
      message.gear = status.gear;
      message.signed_speed_kph = status.signed_speed_kph;
      status_publisher_.publish(message);
    }
  }

  ros::NodeHandle node_;
  ros::NodeHandle private_node_;
  std::unique_ptr<UdpReceiver> receiver_;
  std::array<std::uint8_t, 2048U> buffer_{};
  ros::Publisher status_publisher_;
  ros::WallTimer poll_timer_;
};

}  // namespace
}  // namespace vehicle_control

int main(int argc, char** argv) {
  ros::init(argc, argv, "morai_vehicle_status_udp_node");
  try {
    vehicle_control::MoraiVehicleStatusUdpNode node;
    ros::spin();
  } catch (const std::exception& error) {
    ROS_FATAL("failed to start MORAI vehicle status UDP receiver: %s",
              error.what());
    return 1;
  }
  return 0;
}
