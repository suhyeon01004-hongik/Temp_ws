#include <cmath>
#include <cstddef>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <std_msgs/ColorRGBA.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

namespace morai_visualization {
namespace {

std_msgs::ColorRGBA color(double red, double green, double blue,
                          double alpha = 1.0) {
  std_msgs::ColorRGBA output;
  output.r = red;
  output.g = green;
  output.b = blue;
  output.a = alpha;
  return output;
}

double squaredDistance(const geometry_msgs::Point& first,
                       const geometry_msgs::Point& second) {
  const double dx = second.x - first.x;
  const double dy = second.y - first.y;
  return dx * dx + dy * dy;
}

double yawFromQuaternion(const geometry_msgs::Quaternion& quaternion) {
  return std::atan2(
      2.0 * (quaternion.w * quaternion.z +
             quaternion.x * quaternion.y),
      1.0 - 2.0 * (quaternion.y * quaternion.y +
                   quaternion.z * quaternion.z));
}

class PathVisualizerNode {
 public:
  PathVisualizerNode() : private_node_("~") {
    private_node_.param<std::string>("global_path_topic", global_path_topic_,
                                     "/global_path");
    private_node_.param<std::string>("local_path_topic", local_path_topic_,
                                     "/local_path");
    private_node_.param<std::string>(
        "localization_topic", localization_topic_,
        "/localization/pose");
    private_node_.param<std::string>(
        "lookahead_point_topic", lookahead_point_topic_,
        "/control/lookahead_point");
    private_node_.param<std::string>(
        "stanley_projection_point_topic", stanley_projection_point_topic_,
        "/control/stanley_projection_point");
    private_node_.param<std::string>(
        "marker_topic", marker_topic_, "/visualization/path");
    private_node_.param<std::string>("frame_id", frame_id_, "map");
    private_node_.param<std::string>("base_frame_id", base_frame_id_,
                                     "base_link");
    private_node_.param("global_line_width", global_line_width_, 0.15);
    private_node_.param("local_line_width", local_line_width_, 0.35);
    private_node_.param("vehicle_origin_diameter",
                        vehicle_origin_diameter_, 0.30);
    private_node_.param("vehicle_origin_label_height",
                        vehicle_origin_label_height_, 0.35);
    private_node_.param("nearest_point_diameter", nearest_point_diameter_,
                        0.7);
    private_node_.param("lookahead_point_diameter",
                        lookahead_point_diameter_, 0.8);
    private_node_.param("stanley_projection_point_diameter",
                        stanley_projection_point_diameter_, 0.65);
    private_node_.param("lookahead_marker_lifetime_sec",
                        lookahead_marker_lifetime_sec_, 0.3);
    private_node_.param("show_global_path_start", show_global_path_start_,
                        true);
    private_node_.param("global_start_diameter", global_start_diameter_, 1.4);
    private_node_.param("global_start_text_height",
                        global_start_text_height_, 1.0);

    if (frame_id_.empty() || base_frame_id_.empty() || marker_topic_.empty() ||
        global_path_topic_.empty() || local_path_topic_.empty() ||
        localization_topic_.empty() || lookahead_point_topic_.empty() ||
        stanley_projection_point_topic_.empty()) {
      throw std::invalid_argument(
          "visualizer topic names and frame IDs must not be empty");
    }
    if (global_line_width_ <= 0.0 || local_line_width_ <= 0.0 ||
        vehicle_origin_diameter_ <= 0.0 ||
        vehicle_origin_label_height_ <= 0.0 ||
        nearest_point_diameter_ <= 0.0 ||
        lookahead_point_diameter_ <= 0.0 ||
        stanley_projection_point_diameter_ <= 0.0 ||
        lookahead_marker_lifetime_sec_ <= 0.0 ||
        global_start_diameter_ <= 0.0 ||
        global_start_text_height_ <= 0.0) {
      throw std::invalid_argument("visualizer sizes must be positive");
    }

    marker_publisher_ =
        node_.advertise<visualization_msgs::MarkerArray>(marker_topic_, 1, true);
    global_subscriber_ =
        node_.subscribe(global_path_topic_, 1,
                        &PathVisualizerNode::handleGlobalPath, this);
    local_subscriber_ =
        node_.subscribe(local_path_topic_, 1,
                        &PathVisualizerNode::handleLocalPath, this);
    localization_subscriber_ =
        node_.subscribe(localization_topic_, 10,
                        &PathVisualizerNode::handleLocalization, this);
    lookahead_subscriber_ =
        node_.subscribe(lookahead_point_topic_, 10,
                        &PathVisualizerNode::handleLookaheadPoint, this);
    stanley_projection_subscriber_ =
        node_.subscribe(stanley_projection_point_topic_, 10,
                        &PathVisualizerNode::handleStanleyProjectionPoint,
                        this);
  }

 private:
  bool acceptedFrame(const std::string& received,
                     const std::string& source) const {
    if (received.empty() || received == frame_id_) {
      return true;
    }
    ROS_WARN_STREAM_THROTTLE(
        5.0, "path visualizer discarded " << source << " in frame '"
                                           << received << "' (expected '"
                                           << frame_id_ << "')");
    return false;
  }

  visualization_msgs::Marker marker(const std::string& marker_namespace,
                                     int id, int type) const {
    visualization_msgs::Marker output;
    output.header.frame_id = frame_id_;
    output.header.stamp = ros::Time::now();
    output.ns = marker_namespace;
    output.id = id;
    output.type = type;
    output.action = visualization_msgs::Marker::ADD;
    output.pose.orientation.w = 1.0;
    return output;
  }

  visualization_msgs::Marker vehicleMarker(
      const std::string& marker_namespace, int id, int type) const {
    visualization_msgs::Marker output;
    output.header.frame_id = base_frame_id_;
    // Vehicle-attached markers must follow the latest base_link TF rather than
    // being frozen at the localization message timestamp.
    output.header.stamp = ros::Time(0);
    output.frame_locked = true;
    output.ns = marker_namespace;
    output.id = id;
    output.type = type;
    output.action = visualization_msgs::Marker::ADD;
    output.pose.orientation.w = 1.0;
    return output;
  }

  visualization_msgs::Marker pathMarker(
      const nav_msgs::Path& path, const std::string& marker_namespace,
      double width, const std_msgs::ColorRGBA& marker_color,
      double z_offset) const {
    visualization_msgs::Marker output =
        marker(marker_namespace, 0, visualization_msgs::Marker::LINE_STRIP);
    output.scale.x = width;
    output.color = marker_color;
    output.points.reserve(path.poses.size());
    for (const geometry_msgs::PoseStamped& pose : path.poses) {
      geometry_msgs::Point point = pose.pose.position;
      point.z += z_offset;
      output.points.push_back(point);
    }
    return output;
  }

  void handleGlobalPath(const nav_msgs::Path::ConstPtr& path) {
    if (!acceptedFrame(path->header.frame_id, "global path")) {
      return;
    }
    global_path_ = *path;
    has_global_path_ = true;
    publish();
  }

  void handleLocalPath(const nav_msgs::Path::ConstPtr& path) {
    if (!acceptedFrame(path->header.frame_id, "local path")) {
      return;
    }
    local_path_ = *path;
    has_local_path_ = true;
    publish();
  }

  void handleLocalization(
      const geometry_msgs::PoseStamped::ConstPtr& pose) {
    if (!acceptedFrame(pose->header.frame_id, "localization pose") ||
        !std::isfinite(pose->pose.position.x) ||
        !std::isfinite(pose->pose.position.y) ||
        !std::isfinite(pose->pose.orientation.x) ||
        !std::isfinite(pose->pose.orientation.y) ||
        !std::isfinite(pose->pose.orientation.z) ||
        !std::isfinite(pose->pose.orientation.w)) {
      return;
    }
    current_pose_ = *pose;
    has_current_position_ = true;
    publish();
  }

  void handleLookaheadPoint(
      const geometry_msgs::PointStamped::ConstPtr& point) {
    if (point->header.frame_id != base_frame_id_ ||
        !std::isfinite(point->point.x) ||
        !std::isfinite(point->point.y) ||
        !std::isfinite(point->point.z)) {
      ROS_WARN_STREAM_THROTTLE(
          5.0, "path visualizer discarded lookahead point in frame '"
                   << point->header.frame_id << "' (expected '"
                   << base_frame_id_ << "')");
      return;
    }
    lookahead_point_ = *point;
    has_lookahead_point_ = true;
    publish();
  }

  void handleStanleyProjectionPoint(
      const geometry_msgs::PointStamped::ConstPtr& point) {
    if (point->header.frame_id != base_frame_id_ ||
        !std::isfinite(point->point.x) ||
        !std::isfinite(point->point.y) ||
        !std::isfinite(point->point.z)) {
      ROS_WARN_STREAM_THROTTLE(
          5.0, "path visualizer discarded Stanley projection point in frame '"
                   << point->header.frame_id << "' (expected '"
                   << base_frame_id_ << "')");
      return;
    }
    stanley_projection_point_ = *point;
    has_stanley_projection_point_ = true;
    publish();
  }

  void appendLookaheadMarker(
      visualization_msgs::MarkerArray* markers) const {
    if (!has_lookahead_point_) {
      return;
    }
    visualization_msgs::Marker target =
        vehicleMarker("lookahead_point", 0,
                      visualization_msgs::Marker::SPHERE);
    target.pose.position = lookahead_point_.point;
    target.pose.position.z += lookahead_point_diameter_ * 0.5;
    target.scale.x = lookahead_point_diameter_;
    target.scale.y = lookahead_point_diameter_;
    target.scale.z = lookahead_point_diameter_;
    target.color = color(0.15, 1.0, 0.25);
    target.lifetime = ros::Duration(lookahead_marker_lifetime_sec_);
    markers->markers.push_back(target);
  }

  void appendStanleyProjectionMarker(
      visualization_msgs::MarkerArray* markers) const {
    if (!has_stanley_projection_point_) {
      return;
    }
    visualization_msgs::Marker target =
        vehicleMarker("stanley_projection_point", 0,
                      visualization_msgs::Marker::SPHERE);
    target.pose.position = stanley_projection_point_.point;
    target.pose.position.z += stanley_projection_point_diameter_ * 0.5;
    target.scale.x = stanley_projection_point_diameter_;
    target.scale.y = stanley_projection_point_diameter_;
    target.scale.z = stanley_projection_point_diameter_;
    target.color = color(0.1, 0.75, 1.0);
    target.lifetime = ros::Duration(lookahead_marker_lifetime_sec_);
    markers->markers.push_back(target);
  }

  void appendVehicleAndNearestMarkers(
      visualization_msgs::MarkerArray* markers) const {
    visualization_msgs::Marker origin =
        vehicleMarker("vehicle_origin", 0,
                      visualization_msgs::Marker::SPHERE);
    origin.scale.x = vehicle_origin_diameter_;
    origin.scale.y = vehicle_origin_diameter_;
    origin.scale.z = vehicle_origin_diameter_;
    origin.color = color(0.95, 0.1, 0.1);
    markers->markers.push_back(origin);

    visualization_msgs::Marker origin_label =
        vehicleMarker("vehicle_origin", 1,
                      visualization_msgs::Marker::TEXT_VIEW_FACING);
    origin_label.pose.position.z =
        vehicle_origin_diameter_ + vehicle_origin_label_height_;
    origin_label.scale.z = vehicle_origin_label_height_;
    origin_label.color = color(1.0, 0.25, 0.2);
    origin_label.text = "REAR AXLE";
    markers->markers.push_back(origin_label);

    visualization_msgs::Marker heading =
        vehicleMarker("vehicle_heading", 0,
                      visualization_msgs::Marker::ARROW);
    heading.pose.position.z = 0.15;
    heading.scale.x = 2.5;
    heading.scale.y = 0.35;
    heading.scale.z = 0.35;
    heading.color = color(0.95, 0.15, 0.1);
    markers->markers.push_back(heading);

    if (!has_global_path_ || global_path_.poses.empty()) {
      return;
    }

    const geometry_msgs::Point& current_point =
        current_pose_.pose.position;
    std::size_t nearest_index = 0U;
    double nearest_squared = std::numeric_limits<double>::infinity();
    for (std::size_t index = 0U; index < global_path_.poses.size(); ++index) {
      const double candidate = squaredDistance(
          current_point, global_path_.poses[index].pose.position);
      if (candidate < nearest_squared) {
        nearest_squared = candidate;
        nearest_index = index;
      }
    }
    const geometry_msgs::Point& nearest_point =
        global_path_.poses[nearest_index].pose.position;

    visualization_msgs::Marker nearest =
        marker("nearest_waypoint", 0, visualization_msgs::Marker::SPHERE);
    nearest.pose.position = nearest_point;
    nearest.pose.position.z = nearest_point_diameter_ * 0.5;
    nearest.scale.x = nearest_point_diameter_;
    nearest.scale.y = nearest_point_diameter_;
    nearest.scale.z = nearest_point_diameter_;
    nearest.color = color(1.0, 0.9, 0.0);
    markers->markers.push_back(nearest);

    visualization_msgs::Marker error_line =
        marker("nearest_waypoint", 1, visualization_msgs::Marker::LINE_LIST);
    error_line.scale.x = 0.10;
    error_line.color = color(1.0, 0.15, 0.75);
    geometry_msgs::Point line_start = current_point;
    geometry_msgs::Point line_end = nearest_point;
    line_start.z = 0.15;
    line_end.z = 0.15;
    error_line.points.push_back(line_start);
    error_line.points.push_back(line_end);
    markers->markers.push_back(error_line);

    visualization_msgs::Marker label =
        marker("nearest_waypoint", 2,
               visualization_msgs::Marker::TEXT_VIEW_FACING);
    label.pose.position = current_point;
    label.pose.position.z = 2.0;
    label.scale.z = 0.9;
    label.color = color(1.0, 1.0, 1.0);
    std::ostringstream text;
    text << "Pose (" << std::fixed << std::setprecision(2)
         << current_point.x << ", " << current_point.y
         << ")  yaw: "
         << yawFromQuaternion(current_pose_.pose.orientation) *
                57.29577951308232
         << " deg  nearest waypoint: " << std::sqrt(nearest_squared)
         << " m";
    label.text = text.str();
    markers->markers.push_back(label);
  }

  void appendGlobalPathStartMarkers(
      visualization_msgs::MarkerArray* markers) const {
    if (!show_global_path_start_ || global_path_.poses.empty()) {
      return;
    }

    const geometry_msgs::Point& start_point =
        global_path_.poses.front().pose.position;

    visualization_msgs::Marker start =
        marker("global_path_start", 0, visualization_msgs::Marker::SPHERE);
    start.pose.position = start_point;
    start.pose.position.z += global_start_diameter_ * 0.5;
    start.scale.x = global_start_diameter_;
    start.scale.y = global_start_diameter_;
    start.scale.z = global_start_diameter_;
    start.color = color(0.1, 1.0, 0.2);
    markers->markers.push_back(start);

    visualization_msgs::Marker label =
        marker("global_path_start", 1,
               visualization_msgs::Marker::TEXT_VIEW_FACING);
    label.pose.position = start_point;
    label.pose.position.z +=
        global_start_diameter_ + global_start_text_height_ * 0.5;
    label.scale.z = global_start_text_height_;
    label.color = color(0.1, 1.0, 0.2);
    label.text = "START";
    markers->markers.push_back(label);
  }

  void publish() const {
    visualization_msgs::MarkerArray markers;
    if (has_global_path_) {
      markers.markers.push_back(
          pathMarker(global_path_, "global_path", global_line_width_,
                     color(0.0, 0.75, 0.9, 0.85), 0.03));
      appendGlobalPathStartMarkers(&markers);
    }
    if (has_local_path_) {
      markers.markers.push_back(
          pathMarker(local_path_, "local_path", local_line_width_,
                     color(1.0, 0.45, 0.0), 0.12));
    }
    if (has_current_position_) {
      appendVehicleAndNearestMarkers(&markers);
    }
    appendLookaheadMarker(&markers);
    appendStanleyProjectionMarker(&markers);
    marker_publisher_.publish(markers);
  }

  ros::NodeHandle node_;
  ros::NodeHandle private_node_;
  std::string global_path_topic_;
  std::string local_path_topic_;
  std::string localization_topic_;
  std::string lookahead_point_topic_;
  std::string stanley_projection_point_topic_;
  std::string marker_topic_;
  std::string frame_id_;
  std::string base_frame_id_;
  double global_line_width_ = 0.15;
  double local_line_width_ = 0.35;
  double vehicle_origin_diameter_ = 0.30;
  double vehicle_origin_label_height_ = 0.35;
  double nearest_point_diameter_ = 0.7;
  double lookahead_point_diameter_ = 0.8;
  double stanley_projection_point_diameter_ = 0.65;
  double lookahead_marker_lifetime_sec_ = 0.3;
  bool show_global_path_start_ = true;
  double global_start_diameter_ = 1.4;
  double global_start_text_height_ = 1.0;
  ros::Publisher marker_publisher_;
  ros::Subscriber global_subscriber_;
  ros::Subscriber local_subscriber_;
  ros::Subscriber localization_subscriber_;
  ros::Subscriber lookahead_subscriber_;
  ros::Subscriber stanley_projection_subscriber_;
  nav_msgs::Path global_path_;
  nav_msgs::Path local_path_;
  geometry_msgs::PoseStamped current_pose_;
  geometry_msgs::PointStamped lookahead_point_;
  geometry_msgs::PointStamped stanley_projection_point_;
  bool has_global_path_ = false;
  bool has_local_path_ = false;
  bool has_current_position_ = false;
  bool has_lookahead_point_ = false;
  bool has_stanley_projection_point_ = false;
};

}  // namespace
}  // namespace morai_visualization

int main(int argc, char** argv) {
  ros::init(argc, argv, "path_visualizer");
  try {
    morai_visualization::PathVisualizerNode visualizer;
    ROS_INFO("MORAI path visualizer is running");
    ros::spin();
  } catch (const std::exception& error) {
    ROS_FATAL_STREAM("failed to start path visualizer: " << error.what());
    return 1;
  }
  return 0;
}
