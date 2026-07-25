#include <ros/ros.h>

#include <cmath>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>

#include "vehicle_control/VehicleCommand.h"
#include "vehicle_control/command_watchdog.hpp"
#include "vehicle_control/control_command.hpp"
#include "vehicle_control/morai_ctrl_packet.hpp"
#include "vehicle_control/udp_sender.hpp"

namespace vehicle_control {
namespace {

class MoraiUdpSenderNode {
 public:
  MoraiUdpSenderNode() : private_node_("~") {
    std::string command_topic;
    std::string destination_ip;
    int destination_port = 9093;
    double send_rate = 50.0;
    double command_timeout = 0.25;
    double safe_brake = 0.5;
    private_node_.param<std::string>(
        "command_topic", command_topic, "/vehicle/manual_command");
    private_node_.param<std::string>(
        "destination_ip", destination_ip, "127.0.0.1");
    private_node_.param("destination_port", destination_port, 9093);
    private_node_.param("send_rate", send_rate, 50.0);
    private_node_.param("command_timeout", command_timeout, 0.25);
    private_node_.param("safe_brake", safe_brake, 0.5);

    if (destination_port < 1 || destination_port > 65535) {
      throw std::invalid_argument("destination_port must be in 1..65535");
    }
    if (!std::isfinite(send_rate) || send_rate <= 0.0) {
      throw std::invalid_argument("send_rate must be positive");
    }

    watchdog_.reset(new CommandWatchdog(
        command_timeout, static_cast<float>(safe_brake)));
    sender_.reset(new UdpSender(
        destination_ip, static_cast<std::uint16_t>(destination_port)));
    command_subscriber_ =
        node_.subscribe(command_topic, 1, &MoraiUdpSenderNode::onCommand,
                        this);
    send_timer_ = node_.createWallTimer(
        ros::WallDuration(1.0 / send_rate),
        &MoraiUdpSenderNode::onSendTimer, this);

    ROS_INFO("MORAI control UDP: %s -> %s:%d at %.1f Hz",
             command_topic.c_str(), destination_ip.c_str(),
             destination_port, send_rate);
  }

 private:
  void onCommand(const vehicle_control::VehicleCommand::ConstPtr& message) {
    latest_command_ =
        ControlCommand(message->accel, message->brake, message->steering, 4U);
    has_command_ = true;
    last_command_time_ = ros::WallTime::now();
  }

  void onSendTimer(const ros::WallTimerEvent&) {
    const ros::WallTime now = ros::WallTime::now();
    const double age =
        has_command_ ? (now - last_command_time_).toSec() : 0.0;
    const ControlCommand selected =
        watchdog_->select(latest_command_, has_command_, age);
    const MoraiCtrlPacket packet = encodeMoraiCtrlPacket(selected);
    try {
      sender_->send(packet.data(), packet.size());
    } catch (const std::exception& error) {
      ROS_ERROR_THROTTLE(1.0, "MORAI control UDP send failed: %s",
                         error.what());
    }
  }

  ros::NodeHandle node_;
  ros::NodeHandle private_node_;
  std::unique_ptr<CommandWatchdog> watchdog_;
  std::unique_ptr<UdpSender> sender_;
  ControlCommand latest_command_;
  bool has_command_{false};
  ros::WallTime last_command_time_;
  ros::Subscriber command_subscriber_;
  ros::WallTimer send_timer_;
};

}  // namespace
}  // namespace vehicle_control

int main(int argc, char** argv) {
  ros::init(argc, argv, "morai_udp_sender_node");
  try {
    vehicle_control::MoraiUdpSenderNode node;
    ros::spin();
  } catch (const std::exception& error) {
    ROS_FATAL("failed to start MORAI UDP sender: %s", error.what());
    return 1;
  }
  return 0;
}
