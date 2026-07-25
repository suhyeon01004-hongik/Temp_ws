#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <tf2_ros/transform_broadcaster.h>

#include "morai_localization/imu_orientation.hpp"

namespace morai_localization {
namespace {

constexpr double kDegreesToRadians = 0.017453292519943295;

void requirePositive(const std::string& name, double value) {
  if (!std::isfinite(value) || value <= 0.0) {
    throw std::invalid_argument(name + " must be finite and positive");
  }
}

class LocalizationFusionNode {
 public:
  LocalizationFusionNode() : private_node_("~") {
    loadConfig();

    pose_publisher_ =
        node_.advertise<geometry_msgs::PoseStamped>(pose_topic_, 20);
    odometry_publisher_ =
        node_.advertise<nav_msgs::Odometry>(odometry_topic_, 20);
    gps_subscriber_ =
        node_.subscribe(gps_topic_, 20,
                        &LocalizationFusionNode::handleGps, this);
    imu_subscriber_ =
        node_.subscribe(imu_topic_, 50,
                        &LocalizationFusionNode::handleImu, this);
  }

 private:
  void loadConfig() {
    private_node_.param<std::string>(
        "gps_local_topic", gps_topic_,
        "/localization/gps/local_point");
    private_node_.param<std::string>(
        "imu_output_topic", imu_topic_, "/localization/imu/data");
    private_node_.param<std::string>(
        "pose_topic", pose_topic_, "/localization/pose");
    private_node_.param<std::string>(
        "odometry_topic", odometry_topic_, "/localization/odometry");
    private_node_.param<std::string>("frame_id", map_frame_id_, "map");
    private_node_.param<std::string>("imu_frame_id", imu_frame_id_,
                                     "imu_link");
    private_node_.param<std::string>("base_frame_id", base_frame_id_,
                                     "base_link");
    private_node_.param<std::string>("tf_child_frame_id", tf_child_frame_id_,
                                     "base_footprint");
    private_node_.param("publish_tf", publish_tf_, true);
    private_node_.param("yaw_sign", yaw_sign_, 1.0);
    private_node_.param("yaw_offset_deg", yaw_offset_deg_, 0.0);
    private_node_.param("max_sensor_skew_sec", max_sensor_skew_sec_, 0.2);
    private_node_.param("max_gps_age_sec", max_gps_age_sec_, 1.0);
    private_node_.param("max_imu_age_sec", max_imu_age_sec_, 0.25);
    private_node_.param("minimum_velocity_dt_sec",
                        minimum_velocity_dt_sec_, 0.001);
    private_node_.param("maximum_velocity_dt_sec",
                        maximum_velocity_dt_sec_, 1.0);

    if (gps_topic_.empty() || imu_topic_.empty() || pose_topic_.empty() ||
        odometry_topic_.empty() || map_frame_id_.empty() ||
        imu_frame_id_.empty() || base_frame_id_.empty() ||
        tf_child_frame_id_.empty()) {
      throw std::invalid_argument(
          "localization topic names and frame IDs must not be empty");
    }
    if (yaw_sign_ != 1.0 && yaw_sign_ != -1.0) {
      throw std::invalid_argument("yaw_sign must be either 1.0 or -1.0");
    }
    if (!std::isfinite(yaw_offset_deg_)) {
      throw std::invalid_argument("yaw_offset_deg must be finite");
    }
    requirePositive("max_sensor_skew_sec", max_sensor_skew_sec_);
    requirePositive("max_gps_age_sec", max_gps_age_sec_);
    requirePositive("max_imu_age_sec", max_imu_age_sec_);
    requirePositive("minimum_velocity_dt_sec", minimum_velocity_dt_sec_);
    requirePositive("maximum_velocity_dt_sec", maximum_velocity_dt_sec_);
    if (maximum_velocity_dt_sec_ <= minimum_velocity_dt_sec_) {
      throw std::invalid_argument(
          "maximum_velocity_dt_sec must exceed minimum_velocity_dt_sec");
    }
  }

  static ros::Time messageStamp(const ros::Time& stamp) {
    return stamp.isZero() ? ros::Time::now() : stamp;
  }

  void handleGps(const geometry_msgs::PointStamped::ConstPtr& point) {
    if (!point->header.frame_id.empty() &&
        point->header.frame_id != map_frame_id_) {
      ROS_WARN_STREAM_THROTTLE(
          5.0, "discarding GPS local point in frame '"
                   << point->header.frame_id << "' (expected '"
                   << map_frame_id_ << "')");
      return;
    }
    if (!std::isfinite(point->point.x) || !std::isfinite(point->point.y) ||
        !std::isfinite(point->point.z)) {
      ROS_WARN_THROTTLE(5.0,
                        "discarding non-finite GPS local point");
      return;
    }

    geometry_msgs::PointStamped current = *point;
    current.header.stamp = messageStamp(current.header.stamp);
    current.header.frame_id = map_frame_id_;
    updateRawVelocity(current);
    gps_ = current;
    has_gps_ = true;
    publishIfReady();
  }

  void updateRawVelocity(const geometry_msgs::PointStamped& current) {
    has_velocity_ = false;
    if (has_previous_gps_) {
      const double dt =
          (current.header.stamp - previous_gps_.header.stamp).toSec();
      if (dt >= minimum_velocity_dt_sec_ &&
          dt <= maximum_velocity_dt_sec_) {
        velocity_map_x_ =
            (current.point.x - previous_gps_.point.x) / dt;
        velocity_map_y_ =
            (current.point.y - previous_gps_.point.y) / dt;
        has_velocity_ =
            std::isfinite(velocity_map_x_) &&
            std::isfinite(velocity_map_y_);
      }
    }
    previous_gps_ = current;
    has_previous_gps_ = true;
  }

  void handleImu(const sensor_msgs::Imu::ConstPtr& imu) {
    if (!imu->header.frame_id.empty() &&
        imu->header.frame_id != imu_frame_id_) {
      ROS_WARN_STREAM_THROTTLE(
          5.0, "discarding normalized IMU message in frame '"
                   << imu->header.frame_id << "' (expected '" << imu_frame_id_
                   << "')");
      return;
    }
    if (!isFinite(imu->angular_velocity) ||
        !isFinite(imu->linear_acceleration)) {
      ROS_WARN_THROTTLE(5.0,
                        "discarding non-finite normalized IMU data");
      return;
    }

    try {
      imu_ = *imu;
      imu_.orientation = normalizeQuaternion(imu->orientation);
    } catch (const std::invalid_argument& error) {
      ROS_WARN_STREAM_THROTTLE(5.0, "discarding invalid normalized IMU: "
                                        << error.what());
      return;
    }
    imu_.header.stamp = messageStamp(imu_.header.stamp);
    imu_.header.frame_id = imu_frame_id_;
    has_imu_ = true;
  }

  bool observationsAreCurrent() const {
    const ros::Time now = ros::Time::now();
    const double gps_age = std::max(0.0, (now - gps_.header.stamp).toSec());
    const double imu_age = std::max(0.0, (now - imu_.header.stamp).toSec());
    const double skew =
        std::abs((gps_.header.stamp - imu_.header.stamp).toSec());
    if (gps_age > max_gps_age_sec_ || imu_age > max_imu_age_sec_ ||
        skew > max_sensor_skew_sec_) {
      ROS_WARN_STREAM_THROTTLE(
          5.0, "waiting for synchronized GPS/IMU (GPS age=" << gps_age
                                                             << " s, IMU age="
                                                             << imu_age
                                                             << " s, skew="
                                                             << skew << " s)");
      return false;
    }
    return true;
  }

  void publishIfReady() {
    if (!has_gps_ || !has_imu_ || !observationsAreCurrent()) {
      return;
    }

    const double raw_yaw = yawFromQuaternion(imu_.orientation);
    const double yaw = normalizeAngle(
        yaw_sign_ * raw_yaw + yaw_offset_deg_ * kDegreesToRadians);
    const geometry_msgs::Quaternion orientation = quaternionFromYaw(yaw);
    const ros::Time stamp =
        gps_.header.stamp > imu_.header.stamp
            ? gps_.header.stamp
            : imu_.header.stamp;

    geometry_msgs::PoseStamped pose;
    pose.header.stamp = stamp;
    pose.header.frame_id = map_frame_id_;
    pose.pose.position = gps_.point;
    pose.pose.orientation = orientation;
    pose_publisher_.publish(pose);

    nav_msgs::Odometry odometry;
    odometry.header = pose.header;
    odometry.child_frame_id = base_frame_id_;
    odometry.pose.pose = pose.pose;
    if (has_velocity_) {
      const double cosine = std::cos(yaw);
      const double sine = std::sin(yaw);
      odometry.twist.twist.linear.x =
          cosine * velocity_map_x_ + sine * velocity_map_y_;
      odometry.twist.twist.linear.y =
          -sine * velocity_map_x_ + cosine * velocity_map_y_;
    }
    odometry.twist.twist.angular.z =
        yaw_sign_ * imu_.angular_velocity.z;
    odometry_publisher_.publish(odometry);

    if (publish_tf_) {
      geometry_msgs::TransformStamped transform;
      transform.header = pose.header;
      transform.child_frame_id = tf_child_frame_id_;
      transform.transform.translation.x = pose.pose.position.x;
      transform.transform.translation.y = pose.pose.position.y;
      transform.transform.translation.z = pose.pose.position.z;
      transform.transform.rotation = orientation;
      transform_broadcaster_.sendTransform(transform);
    }
  }

  ros::NodeHandle node_;
  ros::NodeHandle private_node_;
  std::string gps_topic_;
  std::string imu_topic_;
  std::string pose_topic_;
  std::string odometry_topic_;
  std::string map_frame_id_;
  std::string imu_frame_id_;
  std::string base_frame_id_;
  std::string tf_child_frame_id_;
  bool publish_tf_ = true;
  double yaw_sign_ = 1.0;
  double yaw_offset_deg_ = 0.0;
  double max_sensor_skew_sec_ = 0.2;
  double max_gps_age_sec_ = 1.0;
  double max_imu_age_sec_ = 0.25;
  double minimum_velocity_dt_sec_ = 0.001;
  double maximum_velocity_dt_sec_ = 1.0;
  ros::Publisher pose_publisher_;
  ros::Publisher odometry_publisher_;
  ros::Subscriber gps_subscriber_;
  ros::Subscriber imu_subscriber_;
  tf2_ros::TransformBroadcaster transform_broadcaster_;
  geometry_msgs::PointStamped gps_;
  geometry_msgs::PointStamped previous_gps_;
  sensor_msgs::Imu imu_;
  bool has_gps_ = false;
  bool has_previous_gps_ = false;
  bool has_imu_ = false;
  bool has_velocity_ = false;
  double velocity_map_x_ = 0.0;
  double velocity_map_y_ = 0.0;
};

}  // namespace
}  // namespace morai_localization

int main(int argc, char** argv) {
  ros::init(argc, argv, "localization_fusion");
  try {
    morai_localization::LocalizationFusionNode fusion;
    ROS_INFO("direct GPS/IMU localization fusion is running (no filter)");
    ros::spin();
  } catch (const std::exception& error) {
    ROS_FATAL_STREAM("failed to start localization fusion: "
                     << error.what());
    return 1;
  }
  return 0;
}
