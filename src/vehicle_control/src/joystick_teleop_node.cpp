#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>

#include <cmath>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>

#include "vehicle_control/VehicleCommand.h"
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
    std::string odometry_topic;
    private_node_.param<std::string>("joy_topic", joy_topic, "/joy");
    private_node_.param<std::string>(
        "command_topic", command_topic, "/vehicle/manual_command");
    private_node_.param<std::string>(
        "odometry_topic", odometry_topic, "/localization/odometry");
    private_node_.param("odometry_timeout", odometry_timeout_, 0.5);
    if (!std::isfinite(odometry_timeout_) || odometry_timeout_ <= 0.0) {
      throw std::invalid_argument("odometry_timeout must be positive");
    }
    gear_selector_.reset(
        new GearSelector(loadGearSelectorConfig(&private_node_)));

    command_publisher_ =
        node_.advertise<vehicle_control::VehicleCommand>(command_topic, 1);
    joy_subscriber_ =
        node_.subscribe(joy_topic, 1, &JoystickTeleopNode::onJoy, this);
    odometry_subscriber_ = node_.subscribe(
        odometry_topic, 1, &JoystickTeleopNode::onOdometry, this);
    ROS_INFO("CYVOX teleop: %s + %s -> %s", joy_topic.c_str(),
             odometry_topic.c_str(), command_topic.c_str());
  }

 private:
  void onOdometry(const nav_msgs::Odometry::ConstPtr& odometry) {
    const double velocity_x = odometry->twist.twist.linear.x;
    const double velocity_y = odometry->twist.twist.linear.y;
    speed_mps_ = std::hypot(velocity_x, velocity_y);
    has_valid_odometry_ = std::isfinite(speed_mps_);
    last_odometry_time_ = ros::WallTime::now();
  }

  void onJoy(const sensor_msgs::Joy::ConstPtr& joy) {
    ControlCommand command;
    std::string error;
    if (!mapper_->map(joy->axes, &command, &error)) {
      ROS_ERROR_THROTTLE(1.0, "cannot map joystick input: %s",
                         error.c_str());
      return;
    }

    const ros::WallTime now = ros::WallTime::now();
    const double odometry_age =
        has_valid_odometry_ ? (now - last_odometry_time_).toSec() : 0.0;
    const bool speed_valid =
        has_valid_odometry_ && odometry_age >= 0.0 &&
        odometry_age <= odometry_timeout_;
    const GearSelectionResult gear_result =
        gear_selector_->update(joy->buttons, speed_valid, speed_mps_);
    switch (gear_result.status) {
      case GearSelectionStatus::kChanged:
        ROS_INFO("CYVOX gear selected: %s", gearName(gear_result.gear));
        break;
      case GearSelectionStatus::kSpeedUnavailable:
        ROS_WARN_THROTTLE(
            1.0, "gear change rejected: odometry speed is unavailable");
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
  double odometry_timeout_{0.5};
  double speed_mps_{0.0};
  bool has_valid_odometry_{false};
  ros::WallTime last_odometry_time_;
  ros::Publisher command_publisher_;
  ros::Subscriber joy_subscriber_;
  ros::Subscriber odometry_subscriber_;
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
