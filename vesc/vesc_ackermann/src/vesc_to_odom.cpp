// Copyright 2020 F1TENTH Foundation
//
// Redistribution and use in source and binary forms, with or without modification, are permitted
// provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice, this list of conditions
//    and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright notice, this list
//    of conditions and the following disclaimer in the documentation and/or other materials
//    provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its contributors may be used
//    to endorse or promote products derived from this software without specific prior
//    written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
// IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
// FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
// CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
// WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY
// WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

// -*- mode:c++; fill-column: 100; -*-

#include "vesc_ackermann/vesc_to_odom.h"

#include <cmath>
#include <string>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_datatypes.h>

namespace vesc_ackermann
{

template <typename T>
inline bool getRequiredParam(const ros::NodeHandle& nh, const std::string& name, T* value);

static inline double shortestAngularDistance(double from, double to)
{
  double d = fmod(to - from + M_PI, 2.0 * M_PI);
  if (d < 0) d += 2.0 * M_PI;
  return d - M_PI;
}

double VescToOdom::normalizeAngle(double a)
{
  while (a > M_PI)  a -= 2.0 * M_PI;
  while (a < -M_PI) a += 2.0 * M_PI;
  return a;
}

VescToOdom::VescToOdom(ros::NodeHandle nh, ros::NodeHandle private_nh) :
  odom_frame_("odom"), base_frame_("base_link"),
  use_servo_cmd_(true), publish_tf_(false), x_(0.0), y_(0.0), yaw_(0.0),
  use_external_yaw_(false), imu_topic_("/imu/data"),
  reset_requires_stop_(true), reset_speed_thresh_(0.05), zero_external_yaw_on_reset_(true),
  has_imu_yaw_(false), imu_yaw_unbiased_(0.0), yaw_bias_(0.0), imu_yaw_rate_(0.0)
{
  // get ROS parameters
  private_nh.param("odom_frame", odom_frame_, odom_frame_);
  private_nh.param("base_frame", base_frame_, base_frame_);
  private_nh.param("use_servo_cmd_to_calc_angular_velocity", use_servo_cmd_, use_servo_cmd_);
  private_nh.param("use_external_yaw", use_external_yaw_, use_external_yaw_);
  private_nh.param("imu_topic", imu_topic_, imu_topic_);
  private_nh.param("reset_requires_stop", reset_requires_stop_, reset_requires_stop_);
  private_nh.param("reset_speed_thresh", reset_speed_thresh_, reset_speed_thresh_);
  private_nh.param("zero_external_yaw_on_reset", zero_external_yaw_on_reset_, zero_external_yaw_on_reset_);

  if (!getRequiredParam(nh, "speed_to_erpm_gain", &speed_to_erpm_gain_))
    return;
  if (!getRequiredParam(nh, "speed_to_erpm_offset", &speed_to_erpm_offset_))
    return;
  if (use_servo_cmd_)
  {
    if (!getRequiredParam(nh, "steering_angle_to_servo_gain", &steering_to_servo_gain_))
      return;
    if (!getRequiredParam(nh, "steering_angle_to_servo_offset", &steering_to_servo_offset_))
      return;
    if (!getRequiredParam(nh, "wheelbase", &wheelbase_))
      return;
  }
  private_nh.param("publish_tf", publish_tf_, publish_tf_);

  // create odom publisher
  odom_pub_ = nh.advertise<nav_msgs::Odometry>("odom", 10);

  // create tf broadcaster
  if (publish_tf_)
  {
    tf_pub_.reset(new tf::TransformBroadcaster);
  }

  // subscribe to vesc state and, optionally, servo command
  vesc_state_sub_ = nh.subscribe("sensors/core", 10, &VescToOdom::vescStateCallback, this);
  if (use_servo_cmd_)
  {
    servo_sub_ = nh.subscribe("sensors/servo_position_command", 10,
                              &VescToOdom::servoCmdCallback, this);
  }

  // subscribe IMU yaw if enabled
  if (use_external_yaw_)
  {
    imu_sub_ = nh.subscribe(imu_topic_, 50, &VescToOdom::imuCallback, this);
  }

  // advertise reset service
  reset_srv_ = private_nh.advertiseService("reset_odom", &VescToOdom::resetOdom, this);
}

void VescToOdom::vescStateCallback(const vesc_msgs::VescStateStamped::ConstPtr& state)
{
  std::lock_guard<std::mutex> lock(mtx_);

  // If relying on servo for angular velocity, ensure we have it (unless we use external yaw)
  if (use_servo_cmd_ && !use_external_yaw_ && !last_servo_cmd_)
    return;

  // convert to engineering units
  double current_speed = (-state->state.speed - speed_to_erpm_offset_) / speed_to_erpm_gain_;
  if (std::fabs(current_speed) < 0.05)
  {
    current_speed = 0.0;
  }

  // compute yaw and angular velocity
  double current_steering_angle = 0.0;
  double current_angular_velocity = 0.0;

  // If we are using external yaw, yaw_ directly follows IMU (with bias removed)
  if (use_external_yaw_ && has_imu_yaw_)
  {
    // yaw_ is set from IMU in imuCallback via bias; but we still compute twist
    // yaw_ will be assigned below after integration step with external yaw
  }
  else if (use_servo_cmd_)
  {
    current_steering_angle =
      (last_servo_cmd_->data - steering_to_servo_offset_) / steering_to_servo_gain_;
    current_angular_velocity = current_speed * tan(current_steering_angle) / wheelbase_;
  }

  // use current state as last state if this is our first time here
  if (!last_state_)
    last_state_ = state;

  // calc elapsed time
  ros::Duration dt_ros = state->header.stamp - last_state_->header.stamp;
  double dt = dt_ros.toSec();
  if (dt < 0.0) dt = 0.0;  // guard

  // heading to use for integration
  double yaw_for_integration = yaw_;
  if (use_external_yaw_ && has_imu_yaw_)
  {
    // Use the latest IMU yaw (unbiased) minus yaw_bias_
    double imu_yaw_biased = normalizeAngle(imu_yaw_unbiased_ - yaw_bias_);
    yaw_for_integration = imu_yaw_biased;
  }

  // propagate odometry (forward Euler)
  double x_dot = current_speed * cos(yaw_for_integration);
  double y_dot = current_speed * sin(yaw_for_integration);
  x_ += x_dot * dt;
  y_ += y_dot * dt;

  // yaw update
  if (use_external_yaw_ && has_imu_yaw_)
  {
    yaw_ = normalizeAngle(yaw_for_integration);  // directly from IMU (with bias)
    // angular velocity from IMU if available
    if (!last_imu_stamp_.isZero())
    {
      // imu_yaw_rate_ is kept updated in imuCallback
      current_angular_velocity = imu_yaw_rate_;
    }
  }
  else if (use_servo_cmd_)
  {
    yaw_ = normalizeAngle(yaw_ + current_angular_velocity * dt);
  }

  // save state for next time
  last_state_ = state;

  // publish odometry message
  nav_msgs::Odometry::Ptr odom(new nav_msgs::Odometry);
  odom->header.frame_id = odom_frame_;
  odom->header.stamp = state->header.stamp;
  odom->child_frame_id = base_frame_;

  // Position
  odom->pose.pose.position.x = x_;
  odom->pose.pose.position.y = y_;
  odom->pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw_);

  // Position uncertainty (상수 값)
  odom->pose.covariance[0]  = 0.2;  ///< x
  odom->pose.covariance[7]  = 0.2;  ///< y
  odom->pose.covariance[35] = 0.4;  ///< yaw

  // Velocity ("in the coordinate frame given by the child_frame_id")
  odom->twist.twist.linear.x = current_speed;
  odom->twist.twist.linear.y = 0.0;
  odom->twist.twist.angular.z = current_angular_velocity;

  if (publish_tf_)
  {
    geometry_msgs::TransformStamped tfm;
    tfm.header.frame_id = odom_frame_;
    tfm.child_frame_id = base_frame_;
    tfm.header.stamp = ros::Time::now();
    tfm.transform.translation.x = x_;
    tfm.transform.translation.y = y_;
    tfm.transform.translation.z = 0.0;
    tfm.transform.rotation = odom->pose.pose.orientation;
    if (ros::ok())
    {
      tf_pub_->sendTransform(tfm);
    }
  }

  if (ros::ok())
  {
    odom_pub_.publish(odom);
  }
}

void VescToOdom::servoCmdCallback(const std_msgs::Float64::ConstPtr& servo)
{
  std::lock_guard<std::mutex> lock(mtx_);
  last_servo_cmd_ = servo;
}

void VescToOdom::imuCallback(const sensor_msgs::Imu::ConstPtr& imu)
{
  std::lock_guard<std::mutex> lock(mtx_);

  // extract yaw from quaternion
  double roll, pitch, yaw;
  tf::Quaternion q;
  tf::quaternionMsgToTF(imu->orientation, q);
  tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
  yaw = normalizeAngle(yaw);

  ros::Time now = imu->header.stamp.isZero() ? ros::Time::now() : imu->header.stamp;

  // compute yaw rate by shortest angular distance
  if (has_imu_yaw_ && !last_imu_stamp_.isZero())
  {
    double dyaw = shortestAngularDistance(imu_yaw_unbiased_, yaw);
    double dt = (now - last_imu_stamp_).toSec();
    if (dt > 1e-3)
      imu_yaw_rate_ = dyaw / dt;
    else
      imu_yaw_rate_ = 0.0;
  }

  imu_yaw_unbiased_ = yaw;
  last_imu_stamp_ = now;
  has_imu_yaw_ = true;
}

bool VescToOdom::resetOdom(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
{
  std::lock_guard<std::mutex> lock(mtx_);

  // Check stop condition if required
  if (reset_requires_stop_)
  {
    double current_speed = 0.0;
    if (last_state_)
      current_speed = (-last_state_->state.speed - speed_to_erpm_offset_) / speed_to_erpm_gain_;
    if (std::fabs(current_speed) > reset_speed_thresh_)
    {
      ROS_WARN("reset_odom requested while moving (|v|=%.3f m/s > %.3f). Ignoring (set ~reset_requires_stop:=false to override).",
               current_speed, reset_speed_thresh_);
      return false;
    }
  }

  x_ = 0.0;
  y_ = 0.0;

  if (use_external_yaw_ && has_imu_yaw_ && zero_external_yaw_on_reset_)
  {
    // Make current IMU yaw the new zero
    yaw_bias_ = imu_yaw_unbiased_;
    yaw_ = 0.0;
  }
  else
  {
    yaw_ = 0.0;
  }

  last_state_.reset();   // avoid large dt on next update
  ROS_INFO("Odometry reset to (0,0,0).");
  return true;
}

template <typename T>
inline bool getRequiredParam(const ros::NodeHandle& nh, const std::string& name, T* value)
{
  if (nh.getParam(name, *value))
    return true;

  ROS_FATAL("VescToOdom: Parameter %s is required.", name.c_str());
  return false;
}

}  // namespace vesc_ackermann