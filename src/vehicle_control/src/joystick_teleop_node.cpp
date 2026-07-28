#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Empty.h>

#include <cmath>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>

#include "vehicle_control/VehicleCommand.h"
#include "vehicle_control/VehicleStatus.h"
#include "vehicle_control/button_edge.hpp"
#include "vehicle_control/control_command.hpp"
#include "vehicle_control/gear_selector.hpp"
#include "vehicle_control/joy_mapper.hpp"

namespace vehicle_control {
namespace {

JoyMappingConfig loadMappingConfig(ros::NodeHandle* private_node) {
  JoyMappingConfig config;
  private_node->param("steering_axis", config.steering_axis, 0);
  private_node->param("brake_axis", config.brake_axis, 2);
  private_node->param("accel_axis", config.accel_axis, 5);
  private_node->param("steering_inverted", config.steering_inverted, false);
  private_node->param("brake_inverted", config.brake_inverted, false);
  private_node->param("accel_inverted", config.accel_inverted, false);
  double deadzone = 0.05;
  private_node->param("steering_deadzone", deadzone, 0.05);
  config.steering_deadzone = static_cast<float>(deadzone);
  return config;
}

GearSelectorConfig loadGearSelectorConfig(ros::NodeHandle* private_node) {
  GearSelectorConfig config;
  private_node->param("drive_button", config.drive_button, 0);
  private_node->param("neutral_button", config.neutral_button, 1);
  private_node->param("reverse_button", config.reverse_button, 2);
  private_node->param("park_button", config.park_button, 3);
  int initial_gear = 4;
  private_node->param("initial_gear", initial_gear, 4);
  config.initial_gear = static_cast<std::uint8_t>(initial_gear);
  private_node->param("maximum_gear_change_speed_mps",
                      config.maximum_change_speed_mps, 0.5);
  return config;
}

const char* gearName(std::uint8_t gear) {
  switch (gear) {
    case vehicle_control::VehicleCommand::GEAR_PARK:
      return "P";
    case vehicle_control::VehicleCommand::GEAR_REVERSE:
      return "R";
    case vehicle_control::VehicleCommand::GEAR_NEUTRAL:
      return "N";
    case vehicle_control::VehicleCommand::GEAR_DRIVE:
      return "D";
    default:
      return "?";
  }
}

class JoystickTeleopNode {
 public:
  JoystickTeleopNode()
      : private_node_("~"),
        mapper_(new JoyMapper(loadMappingConfig(&private_node_))) {
    std::string joy_topic;
    std::string command_topic;
    std::string status_topic;
    std::string reset_topic;
    private_node_.param<std::string>("joy_topic", joy_topic, "/joy");
    private_node_.param<std::string>(
        "command_topic", command_topic, "/vehicle/manual_command");
    private_node_.param<std::string>(
        "status_topic", status_topic, "/vehicle/status");
    private_node_.param<std::string>(
        "reset_topic", reset_topic, "/vehicle/reset_request");
    private_node_.param("status_timeout", status_timeout_, 0.5);
    if (!std::isfinite(status_timeout_) || status_timeout_ <= 0.0) {
      throw std::invalid_argument("status_timeout must be positive");
    }
    private_node_.param(
        "allow_brake_interlock_without_status",
        allow_brake_interlock_without_status_, true);
    private_node_.param(
        "minimum_brake_for_gear_change",
        minimum_brake_for_gear_change_, 0.5);
    private_node_.param(
        "maximum_accel_for_gear_change",
        maximum_accel_for_gear_change_, 0.05);
    if (!std::isfinite(minimum_brake_for_gear_change_) ||
        minimum_brake_for_gear_change_ < 0.0 ||
        minimum_brake_for_gear_change_ > 1.0) {
      throw std::invalid_argument(
          "minimum_brake_for_gear_change must be in 0..1");
    }
    if (!std::isfinite(maximum_accel_for_gear_change_) ||
        maximum_accel_for_gear_change_ < 0.0 ||
        maximum_accel_for_gear_change_ > 1.0) {
      throw std::invalid_argument(
          "maximum_accel_for_gear_change must be in 0..1");
    }
    int reset_button = 8;
    private_node_.param("reset_button", reset_button, 8);
    gear_selector_.reset(
        new GearSelector(loadGearSelectorConfig(&private_node_)));
    reset_button_edge_.reset(new ButtonEdge(reset_button));

    command_publisher_ =
        node_.advertise<vehicle_control::VehicleCommand>(command_topic, 1);
    reset_publisher_ = node_.advertise<std_msgs::Empty>(reset_topic, 1);
    joy_subscriber_ =
        node_.subscribe(joy_topic, 1, &JoystickTeleopNode::onJoy, this);
    status_subscriber_ = node_.subscribe(
        status_topic, 10, &JoystickTeleopNode::onStatus, this);
    ROS_INFO("CYVOX teleop: %s + %s -> %s; reset -> %s",
             joy_topic.c_str(), status_topic.c_str(), command_topic.c_str(),
             reset_topic.c_str());
  }

 private:
  void onStatus(const vehicle_control::VehicleStatus::ConstPtr& status) {
    speed_mps_ = static_cast<double>(status->signed_speed_kph) / 3.6;
    has_valid_status_ = std::isfinite(speed_mps_);
    last_status_time_ = ros::WallTime::now();
  }

  void onJoy(const sensor_msgs::Joy::ConstPtr& joy) {
    const ButtonEdgeResult reset_result =
        reset_button_edge_->update(joy->buttons);
    if (reset_result == ButtonEdgeResult::kRisingEdge) {
      reset_publisher_.publish(std_msgs::Empty{});
      ROS_INFO("CYVOX Home button requested MORAI reset");
    } else if (reset_result == ButtonEdgeResult::kInvalidButtonMessage) {
      ROS_ERROR_THROTTLE(
          1.0, "cannot read reset button: Joy button array is too short");
    }

    ControlCommand command;
    std::string error;
    if (!mapper_->map(joy->axes, &command, &error)) {
      ROS_ERROR_THROTTLE(1.0, "cannot map joystick input: %s",
                         error.c_str());
      return;
    }

    const ros::WallTime now = ros::WallTime::now();
    const double status_age =
        has_valid_status_ ? (now - last_status_time_).toSec() : 0.0;
    const bool status_is_fresh =
        has_valid_status_ && status_age >= 0.0 &&
        status_age <= status_timeout_;
    const bool brake_interlock_is_satisfied =
        allow_brake_interlock_without_status_ && !status_is_fresh &&
        command.brake >= minimum_brake_for_gear_change_ &&
        command.accel <= maximum_accel_for_gear_change_;
    const bool speed_valid =
        status_is_fresh || brake_interlock_is_satisfied;
    const double gear_change_speed =
        status_is_fresh ? speed_mps_ : 0.0;
    const GearSelectionResult gear_result =
        gear_selector_->update(
            joy->buttons, speed_valid, gear_change_speed);
    switch (gear_result.status) {
      case GearSelectionStatus::kChanged:
        if (brake_interlock_is_satisfied) {
          ROS_INFO(
              "CYVOX gear selected with brake interlock: %s",
              gearName(gear_result.gear));
        } else {
          ROS_INFO("CYVOX gear selected: %s", gearName(gear_result.gear));
        }
        break;
      case GearSelectionStatus::kSpeedUnavailable:
        ROS_WARN_THROTTLE(
            1.0,
            "gear change rejected: MORAI vehicle speed is unavailable; "
            "release RT and hold LT to change gear");
        break;
      case GearSelectionStatus::kTooFast:
        ROS_WARN_THROTTLE(
            1.0, "gear change rejected: vehicle speed is %.3f m/s",
            speed_mps_);
        break;
      case GearSelectionStatus::kAmbiguousButtons:
        ROS_WARN_THROTTLE(
            1.0, "gear change rejected: multiple gear buttons are pressed");
        break;
      case GearSelectionStatus::kInvalidButtonMessage:
        ROS_ERROR_THROTTLE(
            1.0, "gear change rejected: Joy button array is too short");
        break;
      case GearSelectionStatus::kNoRequest:
        break;
    }

    vehicle_control::VehicleCommand message;
    message.header = joy->header;
    message.accel = command.accel;
    message.brake = command.brake;
    message.steering = command.steering;
    message.gear = gear_result.gear;
    command_publisher_.publish(message);
  }

  ros::NodeHandle node_;
  ros::NodeHandle private_node_;
  std::unique_ptr<JoyMapper> mapper_;
  std::unique_ptr<GearSelector> gear_selector_;
  std::unique_ptr<ButtonEdge> reset_button_edge_;
  double status_timeout_{0.5};
  bool allow_brake_interlock_without_status_{true};
  double minimum_brake_for_gear_change_{0.5};
  double maximum_accel_for_gear_change_{0.05};
  double speed_mps_{0.0};
  bool has_valid_status_{false};
  ros::WallTime last_status_time_;
  ros::Publisher command_publisher_;
  ros::Publisher reset_publisher_;
  ros::Subscriber joy_subscriber_;
  ros::Subscriber status_subscriber_;
};

}  // namespace
}  // namespace vehicle_control

int main(int argc, char** argv) {
  ros::init(argc, argv, "joystick_teleop_node");
  try {
    vehicle_control::JoystickTeleopNode node;
    ros::spin();
  } catch (const std::exception& error) {
    ROS_FATAL("failed to start joystick teleop: %s", error.what());
    return 1;
  }
  return 0;
}
