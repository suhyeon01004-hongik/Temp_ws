#include <cmath>
#include <cstdint>
#include <exception>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>

#include <ros/names.h>
#include <ros/ros.h>

#include "morai_udp_bridge/ActuatorCommand.h"
#include "morai_udp_bridge/control_protocol.hpp"
#include "morai_udp_bridge/control_watchdog.hpp"
#include "morai_udp_bridge/udp_sender.hpp"

namespace morai_udp_bridge {
namespace {

float finiteFloat(double value, const char* parameter) {
  if (!std::isfinite(value) ||
      value > static_cast<double>(std::numeric_limits<float>::max()) ||
      value < -static_cast<double>(std::numeric_limits<float>::max())) {
    throw std::invalid_argument(std::string(parameter) +
                                " must be finite and representable as float");
  }
  return static_cast<float>(value);
}

void requirePositiveFinite(double value, const char* parameter) {
  if (!std::isfinite(value) || value <= 0.0) {
    throw std::invalid_argument(std::string(parameter) +
                                " must be finite and positive");
  }
}

template <typename Value>
void loadOptionalParameter(const ros::NodeHandle& node, const char* name,
                           Value* value) {
  if (node.hasParam(name) && !node.getParam(name, *value)) {
    throw std::invalid_argument(std::string(name) +
                                " has an invalid parameter type");
  }
}

}  // namespace

class ControlSenderNode {
 public:
  ControlSenderNode() : node_(), private_node_("~") {
    std::string command_topic = "/control/actuator_command";
    std::string destination_ip = "127.0.0.1";
    int destination_port = 9093;
    double send_rate_hz = 50.0;
    double command_timeout_sec = 0.25;
    double safe_brake_command = 0.50;
    double maximum_steering_angle_deg = 40.0;
    double steering_sign = 1.0;
    int drive_gear = 4;

    loadOptionalParameter(private_node_, "command_topic", &command_topic);
    loadOptionalParameter(private_node_, "destination_ip", &destination_ip);
    loadOptionalParameter(private_node_, "destination_port", &destination_port);
    loadOptionalParameter(private_node_, "send_rate_hz", &send_rate_hz);
    loadOptionalParameter(private_node_, "command_timeout_sec",
                          &command_timeout_sec);
    loadOptionalParameter(private_node_, "safe_brake_command",
                          &safe_brake_command);
    loadOptionalParameter(private_node_, "maximum_steering_angle_deg",
                          &maximum_steering_angle_deg);
    loadOptionalParameter(private_node_, "steering_sign", &steering_sign);
    loadOptionalParameter(private_node_, "drive_gear", &drive_gear);

    std::string name_error;
    if (command_topic.empty() || !ros::names::validate(command_topic, name_error)) {
      throw std::invalid_argument("command_topic is not a valid ROS name: " +
                                  name_error);
    }
    if (destination_port < 1 || destination_port > 65535) {
      throw std::invalid_argument("destination_port must be in 1..65535");
    }
    requirePositiveFinite(send_rate_hz, "send_rate_hz");
    requirePositiveFinite(maximum_steering_angle_deg,
                          "maximum_steering_angle_deg");
    if (steering_sign != -1.0 && steering_sign != 1.0) {
      throw std::invalid_argument("steering_sign must be either -1 or 1");
    }
    if (drive_gear < 1 || drive_gear > 5) {
      throw std::invalid_argument("drive_gear must be in 1..5");
    }

    constexpr double kDegreesToRadians = 0.017453292519943295;
    protocol_config_.maximum_steering_angle_rad = finiteFloat(
        maximum_steering_angle_deg * kDegreesToRadians,
        "maximum_steering_angle_deg");
    protocol_config_.steering_sign = finiteFloat(steering_sign, "steering_sign");
    protocol_config_.drive_gear = static_cast<std::uint8_t>(drive_gear);

    const float safe_brake = finiteFloat(safe_brake_command,
                                         "safe_brake_command");
    watchdog_.reset(new ControlWatchdog(command_timeout_sec, safe_brake));

    std::string protocol_error;
    if (!isValidControlInput(ControlInput{}, protocol_config_, &protocol_error)) {
      throw std::invalid_argument("invalid control protocol configuration: " +
                                  protocol_error);
    }

    sender_.reset(new UdpSender(
        destination_ip, static_cast<std::uint16_t>(destination_port)));
    command_subscriber_ = node_.subscribe(
        command_topic, 1U, &ControlSenderNode::onCommand, this);
    send_timer_ = node_.createWallTimer(
        ros::WallDuration(1.0 / send_rate_hz), &ControlSenderNode::onSendTimer,
        this);
  }

  ControlSenderNode(const ControlSenderNode&) = delete;
  ControlSenderNode& operator=(const ControlSenderNode&) = delete;

 private:
  void onCommand(const ActuatorCommand::ConstPtr& message) {
    const ControlInput candidate{message->accel, message->brake,
                                 message->steering_angle_rad};
    std::string error;
    if (!isValidControlInput(candidate, protocol_config_, &error)) {
      ROS_WARN_THROTTLE(1.0, "discarding invalid actuator command: %s",
                        error.c_str());
      return;
    }

    latest_command_ = candidate;
    has_command_ = true;
    receipt_time_ = ros::WallTime::now();
  }

  void onSendTimer(const ros::WallTimerEvent&) {
    const double receipt_age_sec =
        has_command_ ? (ros::WallTime::now() - receipt_time_).toSec()
                     : std::numeric_limits<double>::infinity();
    const ControlInput selected =
        watchdog_->select(latest_command_, has_command_, receipt_age_sec);

    try {
      const MoraiControlPacket packet =
          encodeMoraiControlPacket(selected, protocol_config_);
      sender_->send(packet.data(), packet.size());
    } catch (const std::exception& error) {
      ROS_WARN_THROTTLE(1.0, "failed to send MORAI control datagram: %s",
                        error.what());
    }
  }

  ros::NodeHandle node_;
  ros::NodeHandle private_node_;
  ControlProtocolConfig protocol_config_;
  std::unique_ptr<ControlWatchdog> watchdog_;
  std::unique_ptr<UdpSender> sender_;
  ControlInput latest_command_;
  bool has_command_{false};
  ros::WallTime receipt_time_;
  ros::Subscriber command_subscriber_;
  ros::WallTimer send_timer_;
};

}  // namespace morai_udp_bridge

int main(int argc, char** argv) {
  ros::init(argc, argv, "control_sender_node");
  try {
    morai_udp_bridge::ControlSenderNode sender;
    ros::spin();
  } catch (const std::exception& error) {
    ROS_FATAL("failed to start MORAI control sender: %s", error.what());
    return 1;
  } catch (...) {
    ROS_FATAL("failed to start MORAI control sender: unknown exception");
    return 1;
  }
  return 0;
}
