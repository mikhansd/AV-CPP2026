#pragma once

#include <memory>
#include <string>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "vehicle_core/msg/drive_feedback.hpp"

namespace vehicle_core {

class AckermannOdometryNode : public rclcpp::Node {
public:
  explicit AckermannOdometryNode(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void onDriveFeedback(const msg::DriveFeedback::SharedPtr feedback);
  void publishOdometry(const rclcpp::Time & stamp, double speed_mps,
                       double yaw_rate_rad_s);
  void publishTransform(const rclcpp::Time & stamp);
  void resetIntegrationTime(const rclcpp::Time & stamp);

  rclcpp::Subscription<msg::DriveFeedback>::SharedPtr feedback_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  std::string feedback_topic_;
  std::string odometry_topic_;
  std::string odom_frame_;
  std::string base_frame_;

  double wheelbase_m_{0.8128};
  double steering_angle_scale_{1.0};
  double steering_angle_offset_rad_{0.0};
  double max_abs_steering_angle_rad_{0.5};
  double speed_deadband_mps_{0.02};
  double maximum_integration_dt_s_{0.25};
  double pose_xy_stddev_m_{0.10};
  double pose_yaw_stddev_rad_{0.10};
  double speed_stddev_mps_{0.15};
  double yaw_rate_stddev_rad_s_{0.15};
  bool publish_tf_{false};

  double x_m_{0.0};
  double y_m_{0.0};
  double yaw_rad_{0.0};
  rclcpp::Time previous_stamp_{0, 0, RCL_ROS_TIME};
  bool have_previous_stamp_{false};
};

}  // namespace vehicle_core
