#include "vehicle_core/ackermann_odometry_node.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <stdexcept>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp_components/register_node_macro.hpp>

namespace vehicle_core {
namespace {

geometry_msgs::msg::Quaternion yawToQuaternion(double yaw)
{
  geometry_msgs::msg::Quaternion q;
  q.x = 0.0;
  q.y = 0.0;
  q.z = std::sin(0.5 * yaw);
  q.w = std::cos(0.5 * yaw);
  return q;
}

double square(double value)
{
  return value * value;
}

}  // namespace

AckermannOdometryNode::AckermannOdometryNode(const rclcpp::NodeOptions & options)
: Node("ackermann_odometry_node", options)
{
  feedback_topic_ = declare_parameter<std::string>(
    "feedback_topic", "/vehicle/drive_feedback");
  odometry_topic_ = declare_parameter<std::string>(
    "odometry_topic", "/wheel/odometry");
  odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
  base_frame_ = declare_parameter<std::string>("base_frame", "base_link");

  wheelbase_m_ = declare_parameter<double>("wheelbase_m", 0.8128);
  steering_angle_scale_ = declare_parameter<double>("steering_angle_scale", 1.0);
  steering_angle_offset_rad_ = declare_parameter<double>(
    "steering_angle_offset_rad", 0.0);
  max_abs_steering_angle_rad_ = declare_parameter<double>(
    "max_abs_steering_angle_rad", 0.5);
  speed_deadband_mps_ = declare_parameter<double>("speed_deadband_mps", 0.02);
  maximum_integration_dt_s_ = declare_parameter<double>(
    "maximum_integration_dt_s", 0.25);
  publish_tf_ = declare_parameter<bool>("publish_tf", false);

  pose_xy_stddev_m_ = declare_parameter<double>("pose_xy_stddev_m", 0.10);
  pose_yaw_stddev_rad_ = declare_parameter<double>("pose_yaw_stddev_rad", 0.10);
  speed_stddev_mps_ = declare_parameter<double>("speed_stddev_mps", 0.15);
  yaw_rate_stddev_rad_s_ = declare_parameter<double>(
    "yaw_rate_stddev_rad_s", 0.15);

  if (wheelbase_m_ <= 0.0) {
    throw std::invalid_argument("wheelbase_m must be greater than zero");
  }
  if (maximum_integration_dt_s_ <= 0.0) {
    throw std::invalid_argument("maximum_integration_dt_s must be greater than zero");
  }

  odometry_pub_ = create_publisher<nav_msgs::msg::Odometry>(odometry_topic_, 20);
  feedback_sub_ = create_subscription<msg::DriveFeedback>(
    feedback_topic_, rclcpp::SensorDataQoS(),
    std::bind(&AckermannOdometryNode::onDriveFeedback, this,
              std::placeholders::_1));

  if (publish_tf_) {
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  }

  RCLCPP_INFO(
    get_logger(),
    "Ackermann odometry ready: wheelbase=%.4f m, input=%s, output=%s, publish_tf=%s",
    wheelbase_m_, feedback_topic_.c_str(), odometry_topic_.c_str(),
    publish_tf_ ? "true" : "false");
}

void AckermannOdometryNode::onDriveFeedback(
  const msg::DriveFeedback::SharedPtr feedback)
{
  const rclcpp::Time stamp = now();
  const double speed_mps = static_cast<double>(feedback->speed_mmps) * 0.001;
  const double measured_steering_rad =
    static_cast<double>(feedback->steer_millirad) * 0.001;
  const double steering_rad = std::clamp(
    measured_steering_rad * steering_angle_scale_ + steering_angle_offset_rad_,
    -max_abs_steering_angle_rad_, max_abs_steering_angle_rad_);

  const double filtered_speed_mps =
    std::abs(speed_mps) < speed_deadband_mps_ ? 0.0 : speed_mps;
  const double yaw_rate_rad_s =
    filtered_speed_mps * std::tan(steering_rad) / wheelbase_m_;

  if (!have_previous_stamp_) {
    resetIntegrationTime(stamp);
    publishOdometry(stamp, filtered_speed_mps, yaw_rate_rad_s);
    return;
  }

  const double dt_s = (stamp - previous_stamp_).seconds();
  previous_stamp_ = stamp;

  if (dt_s <= 0.0 || dt_s > maximum_integration_dt_s_) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Skipping odometry integration because dt=%.3f s is outside (0, %.3f] s",
      dt_s, maximum_integration_dt_s_);
    publishOdometry(stamp, filtered_speed_mps, yaw_rate_rad_s);
    return;
  }

  // Midpoint integration is more accurate than Euler integration while turning.
  const double delta_yaw = yaw_rate_rad_s * dt_s;
  const double midpoint_yaw = yaw_rad_ + 0.5 * delta_yaw;
  const double distance_m = filtered_speed_mps * dt_s;
  x_m_ += distance_m * std::cos(midpoint_yaw);
  y_m_ += distance_m * std::sin(midpoint_yaw);
  yaw_rad_ = std::atan2(std::sin(yaw_rad_ + delta_yaw),
                        std::cos(yaw_rad_ + delta_yaw));

  publishOdometry(stamp, filtered_speed_mps, yaw_rate_rad_s);
}

void AckermannOdometryNode::publishOdometry(
  const rclcpp::Time & stamp, double speed_mps, double yaw_rate_rad_s)
{
  nav_msgs::msg::Odometry odom;
  odom.header.stamp = stamp;
  odom.header.frame_id = odom_frame_;
  odom.child_frame_id = base_frame_;

  odom.pose.pose.position.x = x_m_;
  odom.pose.pose.position.y = y_m_;
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation = yawToQuaternion(yaw_rad_);

  odom.twist.twist.linear.x = speed_mps;
  odom.twist.twist.angular.z = yaw_rate_rad_s;

  // Unknown 3-D axes get deliberately large variances in this planar model.
  odom.pose.covariance[0] = square(pose_xy_stddev_m_);
  odom.pose.covariance[7] = square(pose_xy_stddev_m_);
  odom.pose.covariance[14] = 1.0e6;
  odom.pose.covariance[21] = 1.0e6;
  odom.pose.covariance[28] = 1.0e6;
  odom.pose.covariance[35] = square(pose_yaw_stddev_rad_);

  odom.twist.covariance[0] = square(speed_stddev_mps_);
  odom.twist.covariance[7] = 1.0e6;
  odom.twist.covariance[14] = 1.0e6;
  odom.twist.covariance[21] = 1.0e6;
  odom.twist.covariance[28] = 1.0e6;
  odom.twist.covariance[35] = square(yaw_rate_stddev_rad_s_);

  odometry_pub_->publish(odom);
  if (publish_tf_) {
    publishTransform(stamp);
  }
}

void AckermannOdometryNode::publishTransform(const rclcpp::Time & stamp)
{
  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = stamp;
  transform.header.frame_id = odom_frame_;
  transform.child_frame_id = base_frame_;
  transform.transform.translation.x = x_m_;
  transform.transform.translation.y = y_m_;
  transform.transform.translation.z = 0.0;
  transform.transform.rotation = yawToQuaternion(yaw_rad_);
  tf_broadcaster_->sendTransform(transform);
}

void AckermannOdometryNode::resetIntegrationTime(const rclcpp::Time & stamp)
{
  previous_stamp_ = stamp;
  have_previous_stamp_ = true;
}

}  // namespace vehicle_core

RCLCPP_COMPONENTS_REGISTER_NODE(vehicle_core::AckermannOdometryNode)
