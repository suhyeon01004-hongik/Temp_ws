#include <ros/ros.h>
#include <std_msgs/Empty.h>

#include <exception>
#include <string>
#include <vector>

#include "vehicle_control/reset_command.hpp"

namespace vehicle_control {

class MoraiSimResetNode {
 public:
  MoraiSimResetNode() : private_node_("~") {
    std::string reset_topic;
    std::string window_name;
    std::string reset_key;
    private_node_.param<std::string>(
        "reset_topic", reset_topic, "/vehicle/reset_request");
    private_node_.param<std::string>(
        "window_name", window_name, "Simulator");
    private_node_.param<std::string>("reset_key", reset_key, "i");

    reset_command_ = buildMoraiResetCommand(window_name, reset_key);
    reset_subscriber_ = node_.subscribe(
        reset_topic, 1, &MoraiSimResetNode::onResetRequest, this);

    ROS_INFO("MORAI reset bridge: %s -> window '%s', key '%s'",
             reset_topic.c_str(), window_name.c_str(), reset_key.c_str());
  }

 private:
  void onResetRequest(const std_msgs::Empty::ConstPtr&) {
    const ProcessResult result = executeCommand(reset_command_);
    if (!result.started) {
      ROS_ERROR_THROTTLE(
          2.0,
          "MORAI reset command could not start: %s. Install xdotool and use "
          "an X11 desktop session.",
          result.error.c_str());
      return;
    }
    if (result.exit_code != 0) {
      ROS_ERROR("MORAI reset command failed with exit code %d%s%s",
                result.exit_code, result.error.empty() ? "" : ": ",
                result.error.c_str());
      return;
    }

    ROS_INFO("MORAI simulator reset key sent");
  }

  ros::NodeHandle node_;
  ros::NodeHandle private_node_;
  ros::Subscriber reset_subscriber_;
  std::vector<std::string> reset_command_;
};

}  // namespace vehicle_control

int main(int argc, char** argv) {
  ros::init(argc, argv, "morai_sim_reset_node");
  try {
    vehicle_control::MoraiSimResetNode node;
    ros::spin();
  } catch (const std::exception& exception) {
    ROS_FATAL("MORAI reset bridge configuration error: %s", exception.what());
    return 1;
  }
  return 0;
}
