#include <cmath>
#include <cstdint>
#include <exception>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <XmlRpcValue.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/names.h>
#include <ros/ros.h>
#include <tf2/utils.h>

#include "morai_path_tracking/pid_controller.hpp"
#include "morai_path_tracking/pure_pursuit.hpp"
#include "morai_udp_bridge/ActuatorCommand.h"

namespace morai_path_tracking {
namespace {

using XmlValue = XmlRpc::XmlRpcValue;

constexpr double kDegreesToRadians = 0.017453292519943295;

void requirePositive(const char* name, double value) {
  if (!std::isfinite(value) || value <= 0.0) {
    throw std::invalid_argument(std::string(name) +
                                " must be finite and positive");
  }
}

void requireNonNegative(const char* name, double value) {
  if (!std::isfinite(value) || value < 0.0) {
    throw std::invalid_argument(std::string(name) +
                                " must be finite and non-negative");
  }
}

XmlValue requiredParameter(const ros::NodeHandle& node, const char* name) {
  XmlValue value;
  if (!node.getParam(name, value)) {
    throw std::invalid_argument(std::string("required private parameter '~") +
                                name + "' is missing");
  }
  return value;
}

std::string requiredString(const ros::NodeHandle& node, const char* name) {
  const XmlValue value = requiredParameter(node, name);
  if (value.getType() != XmlValue::TypeString) {
    throw std::invalid_argument(std::string("~") + name +
                                " must be a string");
  }
  return static_cast<std::string>(value);
}

double requiredDouble(const ros::NodeHandle& node, const char* name) {
  const XmlValue value = requiredParameter(node, name);
  if (value.getType() == XmlValue::TypeDouble) {
    return static_cast<double>(value);
  }
  if (value.getType() == XmlValue::TypeInt) {
    return static_cast<int>(value);
  }
  throw std::invalid_argument(std::string("~") + name + " must be numeric");
}

void requireRosName(const char* name, const std::string& value) {
  std::string error;
  if (value.empty() || !ros::names::validate(value, error)) {
    throw std::invalid_argument(std::string(name) +
                                " must be a valid ROS name: " + error);
  }
}

ros::WallDuration periodFromRate(double control_rate_hz) {
  requirePositive("control_rate_hz", control_rate_hz);
  const double period_sec = 1.0 / control_rate_hz;
  if (!std::isfinite(period_sec) || period_sec <= 0.0 ||
      period_sec > static_cast<double>(std::numeric_limits<std::int32_t>::max())) {
    throw std::invalid_argument(
        "control_rate_hz produces an unrepresentable WallTimer period");
  }
  const ros::WallDuration period(period_sec);
  if (period.toNSec() <= 0) {
    throw std::invalid_argument(
        "control_rate_hz produces a non-positive WallTimer period");
  }
  return period;
}

struct ControllerConfig {
  std::string local_path_topic;
  std::string odometry_topic;
  std::string command_topic;
  std::string expected_frame_id;
  ros::WallDuration control_period;
  double path_timeout_sec{0.25};
  double odometry_timeout_sec{0.25};
  double maximum_input_skew_sec{0.10};
  double safe_brake_command{0.50};
  double target_speed_mps{3.0};
  PurePursuitConfig pure_pursuit;
  PidConfig pid;
};

ControllerConfig loadConfig(const ros::NodeHandle& private_node) {
  ControllerConfig config;
  config.local_path_topic = requiredString(private_node, "local_path_topic");
  config.odometry_topic = requiredString(private_node, "odometry_topic");
  config.command_topic = requiredString(private_node, "command_topic");
  config.expected_frame_id = requiredString(private_node, "expected_frame_id");
  const double control_rate_hz =
      requiredDouble(private_node, "control_rate_hz");
  config.path_timeout_sec = requiredDouble(private_node, "path_timeout_sec");
  config.odometry_timeout_sec =
      requiredDouble(private_node, "odometry_timeout_sec");
  config.maximum_input_skew_sec =
      requiredDouble(private_node, "maximum_input_skew_sec");
  config.safe_brake_command =
      requiredDouble(private_node, "safe_brake_command");
  config.pure_pursuit.wheelbase_m =
      requiredDouble(private_node, "wheelbase_m");
  config.pure_pursuit.lookahead_base_m =
      requiredDouble(private_node, "lookahead_base_m");
  config.pure_pursuit.lookahead_speed_gain_sec =
      requiredDouble(private_node, "lookahead_speed_gain_sec");
  config.pure_pursuit.lookahead_min_m =
      requiredDouble(private_node, "lookahead_min_m");
  config.pure_pursuit.lookahead_max_m =
      requiredDouble(private_node, "lookahead_max_m");
  config.pure_pursuit.minimum_target_distance_m =
      requiredDouble(private_node, "minimum_target_distance_m");
  const double maximum_steering_angle_deg =
      requiredDouble(private_node, "maximum_steering_angle_deg");
  config.target_speed_mps = requiredDouble(private_node, "target_speed_mps");
  config.pid.kp = requiredDouble(private_node, "speed_kp");
  config.pid.ki = requiredDouble(private_node, "speed_ki");
  config.pid.kd = requiredDouble(private_node, "speed_kd");
  config.pid.integral_limit =
      requiredDouble(private_node, "speed_integral_limit");
  config.pid.error_deadband_mps =
      requiredDouble(private_node, "speed_error_deadband_mps");
  config.pid.maximum_accel =
      requiredDouble(private_node, "maximum_accel_command");
  config.pid.maximum_brake =
      requiredDouble(private_node, "maximum_brake_command");

  requireRosName("local_path_topic", config.local_path_topic);
  requireRosName("odometry_topic", config.odometry_topic);
  requireRosName("command_topic", config.command_topic);
  if (config.expected_frame_id.empty()) {
    throw std::invalid_argument("expected_frame_id must not be empty");
  }
  config.control_period = periodFromRate(control_rate_hz);
  requirePositive("path_timeout_sec", config.path_timeout_sec);
  requirePositive("odometry_timeout_sec", config.odometry_timeout_sec);
  requireNonNegative("maximum_input_skew_sec", config.maximum_input_skew_sec);
  if (config.safe_brake_command < 0.0 || config.safe_brake_command > 1.0) {
    throw std::invalid_argument("safe_brake_command must be in [0, 1]");
  }
  requirePositive("maximum_steering_angle_deg", maximum_steering_angle_deg);
  if (maximum_steering_angle_deg >= 90.0) {
    throw std::invalid_argument("maximum_steering_angle_deg must be below 90");
  }
  config.pure_pursuit.maximum_steering_angle_rad =
      maximum_steering_angle_deg * kDegreesToRadians;
  requireNonNegative("target_speed_mps", config.target_speed_mps);
  if (config.pid.maximum_accel > 1.0 || config.pid.maximum_brake > 1.0) {
    throw std::invalid_argument(
        "maximum_accel_command and maximum_brake_command must be in [0, 1]");
  }

  // The core constructors enforce all finite/range constraints and the
  // Pure Pursuit lookahead cross-field relation before the timer starts.
  LongitudinalPid pid_validation(config.pid);
  (void)pid_validation;
  (void)computePurePursuit({}, 0.0, config.pure_pursuit);
  return config;
}

bool finiteQuaternion(const geometry_msgs::Quaternion& quaternion) {
  return std::isfinite(quaternion.x) && std::isfinite(quaternion.y) &&
         std::isfinite(quaternion.z) && std::isfinite(quaternion.w) &&
         std::isfinite(quaternion.x * quaternion.x +
                       quaternion.y * quaternion.y +
                       quaternion.z * quaternion.z +
                       quaternion.w * quaternion.w) &&
         (quaternion.x * quaternion.x + quaternion.y * quaternion.y +
              quaternion.z * quaternion.z + quaternion.w * quaternion.w) >
             0.0;
}

}  // namespace

class PurePursuitControllerNode {
 public:
  PurePursuitControllerNode()
      : private_node_("~"),
        config_(loadConfig(private_node_)),
        pid_(config_.pid) {
    publisher_ = node_.advertise<morai_udp_bridge::ActuatorCommand>(
        config_.command_topic, 1U);
    path_subscriber_ = node_.subscribe(config_.local_path_topic, 1U,
                                        &PurePursuitControllerNode::onPath,
                                        this);
    odometry_subscriber_ = node_.subscribe(
        config_.odometry_topic, 1U, &PurePursuitControllerNode::onOdometry,
        this);
    timer_ = node_.createWallTimer(config_.control_period,
                                   &PurePursuitControllerNode::onTimer, this);
  }

 private:
  void onPath(const nav_msgs::Path::ConstPtr& message) {
    latest_path_ = message;
    path_receipt_time_ = ros::WallTime::now();
  }

  void onOdometry(const nav_msgs::Odometry::ConstPtr& message) {
    latest_odometry_ = message;
    odometry_receipt_time_ = ros::WallTime::now();
  }

  bool validInputs(const ros::WallTime& now, std::vector<Point2d>* path,
                   double* speed_mps) const {
    if (!latest_path_ || !latest_odometry_) {
      return false;
    }
    const nav_msgs::Path& path_message = *latest_path_;
    const nav_msgs::Odometry& odometry_message = *latest_odometry_;
    if (path_message.header.frame_id != config_.expected_frame_id ||
        odometry_message.header.frame_id != config_.expected_frame_id ||
        path_message.header.stamp.isZero() ||
        odometry_message.header.stamp.isZero()) {
      return false;
    }

    const double path_receipt_age = (now - path_receipt_time_).toSec();
    const double odometry_receipt_age =
        (now - odometry_receipt_time_).toSec();
    if (!std::isfinite(path_receipt_age) || !std::isfinite(odometry_receipt_age) ||
        path_receipt_age < 0.0 || odometry_receipt_age < 0.0 ||
        path_receipt_age > config_.path_timeout_sec ||
        odometry_receipt_age > config_.odometry_timeout_sec) {
      return false;
    }

    const ros::Time ros_now = ros::Time::now();
    const double path_stamp_age = (ros_now - path_message.header.stamp).toSec();
    const double odometry_stamp_age =
        (ros_now - odometry_message.header.stamp).toSec();
    const double skew =
        std::abs((path_message.header.stamp - odometry_message.header.stamp).toSec());
    if (!std::isfinite(path_stamp_age) || !std::isfinite(odometry_stamp_age) ||
        !std::isfinite(skew) || path_stamp_age < 0.0 ||
        odometry_stamp_age < 0.0 || path_stamp_age > config_.path_timeout_sec ||
        odometry_stamp_age > config_.odometry_timeout_sec ||
        skew > config_.maximum_input_skew_sec) {
      return false;
    }

    const geometry_msgs::Point& position = odometry_message.pose.pose.position;
    const geometry_msgs::Quaternion& orientation =
        odometry_message.pose.pose.orientation;
    const double speed = odometry_message.twist.twist.linear.x;
    if (!std::isfinite(position.x) || !std::isfinite(position.y) ||
        !std::isfinite(position.z) || !finiteQuaternion(orientation) ||
        !std::isfinite(speed)) {
      return false;
    }
    const double yaw = tf2::getYaw(orientation);
    if (!std::isfinite(yaw)) {
      return false;
    }

    path->clear();
    path->reserve(path_message.poses.size());
    const double cos_yaw = std::cos(yaw);
    const double sin_yaw = std::sin(yaw);
    for (const geometry_msgs::PoseStamped& pose : path_message.poses) {
      const double point_x = pose.pose.position.x;
      const double point_y = pose.pose.position.y;
      if (!std::isfinite(point_x) || !std::isfinite(point_y)) {
        return false;
      }
      const double dx = point_x - position.x;
      const double dy = point_y - position.y;
      const double x_body = cos_yaw * dx + sin_yaw * dy;
      const double y_body = -sin_yaw * dx + cos_yaw * dy;
      if (!std::isfinite(x_body) || !std::isfinite(y_body)) {
        return false;
      }
      path->push_back({x_body, y_body});
    }
    *speed_mps = speed;
    return true;
  }

  void publishSafe() {
    pid_.reset();
    morai_udp_bridge::ActuatorCommand output;
    output.header.stamp = ros::Time::now();
    output.accel = 0.0F;
    output.brake = static_cast<float>(config_.safe_brake_command);
    output.steering_angle_rad = 0.0F;
    publisher_.publish(output);
  }

  void onTimer(const ros::WallTimerEvent&) {
    const ros::WallTime now = ros::WallTime::now();
    const double dt_sec = has_last_timer_time_
                              ? (now - last_timer_time_).toSec()
                              : std::numeric_limits<double>::quiet_NaN();
    last_timer_time_ = now;
    has_last_timer_time_ = true;
    if (!std::isfinite(dt_sec) || dt_sec <= 0.0) {
      publishSafe();
      return;
    }

    std::vector<Point2d> vehicle_path;
    double speed_mps = 0.0;
    if (!validInputs(now, &vehicle_path, &speed_mps)) {
      publishSafe();
      return;
    }

    try {
      const PurePursuitResult lateral =
          computePurePursuit(vehicle_path, speed_mps, config_.pure_pursuit);
      if (!lateral.valid || !std::isfinite(lateral.steering_angle_rad)) {
        publishSafe();
        return;
      }
      const LongitudinalCommand longitudinal =
          pid_.update(config_.target_speed_mps, speed_mps, dt_sec);
      if (!std::isfinite(longitudinal.accel) ||
          !std::isfinite(longitudinal.brake) || longitudinal.accel < 0.0 ||
          longitudinal.brake < 0.0 ||
          (longitudinal.accel > 0.0 && longitudinal.brake > 0.0)) {
        publishSafe();
        return;
      }

      morai_udp_bridge::ActuatorCommand output;
      output.header.stamp = ros::Time::now();
      output.accel = static_cast<float>(longitudinal.accel);
      output.brake = static_cast<float>(longitudinal.brake);
      output.steering_angle_rad =
          static_cast<float>(lateral.steering_angle_rad);
      publisher_.publish(output);
    } catch (const std::exception& error) {
      ROS_WARN_THROTTLE(1.0, "controller cycle rejected: %s", error.what());
      publishSafe();
    }
  }

  ros::NodeHandle node_;
  ros::NodeHandle private_node_;
  ControllerConfig config_;
  LongitudinalPid pid_;
  ros::Publisher publisher_;
  ros::Subscriber path_subscriber_;
  ros::Subscriber odometry_subscriber_;
  ros::WallTimer timer_;
  nav_msgs::Path::ConstPtr latest_path_;
  nav_msgs::Odometry::ConstPtr latest_odometry_;
  ros::WallTime path_receipt_time_;
  ros::WallTime odometry_receipt_time_;
  ros::WallTime last_timer_time_;
  bool has_last_timer_time_{false};
};

}  // namespace morai_path_tracking

int main(int argc, char** argv) {
  ros::init(argc, argv, "pure_pursuit_controller_node");
  try {
    morai_path_tracking::PurePursuitControllerNode node;
    ros::spin();
  } catch (const std::exception& error) {
    ROS_FATAL("failed to start pure pursuit controller: %s", error.what());
    return 1;
  } catch (...) {
    ROS_FATAL("failed to start pure pursuit controller: unknown exception");
    return 1;
  }
  return 0;
}
