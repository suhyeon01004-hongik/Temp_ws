#include <ros/ros.h>
#include <sensor_msgs/Joy.h>

#include <memory>
#include <stdexcept>
#include <string>

#include "vehicle_control/VehicleCommand.h"
#include "vehicle_control/control_command.hpp"
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

class JoystickTeleopNode {
 public:
  JoystickTeleopNode()
      : private_node_("~"),
        mapper_(new JoyMapper(loadMappingConfig(&private_node_))) {
    std::string joy_topic;
    std::string command_topic;
    private_node_.param<std::string>("joy_topic", joy_topic, "/joy");
    private_node_.param<std::string>(
        "command_topic", command_topic, "/vehicle/manual_command");

    command_publisher_ =
        node_.advertise<vehicle_control::VehicleCommand>(command_topic, 1);
    joy_subscriber_ =
        node_.subscribe(joy_topic, 1, &JoystickTeleopNode::onJoy, this);
    ROS_INFO("CYVOX teleop: %s -> %s", joy_topic.c_str(),
             command_topic.c_str());
  }

 private:
  void onJoy(const sensor_msgs::Joy::ConstPtr& joy) {
    ControlCommand command;
    std::string error;
    if (!mapper_->map(joy->axes, &command, &error)) {
      ROS_ERROR_THROTTLE(1.0, "cannot map joystick input: %s",
                         error.c_str());
      return;
    }

    vehicle_control::VehicleCommand message;
    message.header = joy->header;
    message.accel = command.accel;
    message.brake = command.brake;
    message.steering = command.steering;
    message.gear = vehicle_control::VehicleCommand::GEAR_DRIVE;
    command_publisher_.publish(message);
  }

  ros::NodeHandle node_;
  ros::NodeHandle private_node_;
  std::unique_ptr<JoyMapper> mapper_;
  ros::Publisher command_publisher_;
  ros::Subscriber joy_subscriber_;
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
