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

#ifndef VESC_ACKERMANN_VESC_TO_ODOM_H_
#define VESC_ACKERMANN_VESC_TO_ODOM_H_

#include <memory>
#include <string>
#include <mutex>

#include <ros/ros.h>
#include <vesc_msgs/VescStateStamped.h>
#include <std_msgs/Float64.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/Imu.h>
#include <std_srvs/Empty.h>

namespace vesc_ackermann
{

class VescToOdom
{
public:
  VescToOdom(ros::NodeHandle nh, ros::NodeHandle private_nh);

private:
  // ROS parameters
  std::string odom_frame_;
  std::string base_frame_;
  /** State message does not report servo position, so use the command instead */
  bool use_servo_cmd_;
  // external yaw (IMU) options
  bool use_external_yaw_;              // 외부 IMU yaw 사용 여부
  std::string imu_topic_;             // IMU 토픽
  bool reset_requires_stop_;          // 정지 상태에서만 리셋 허용
  double reset_speed_thresh_;         // 정지 임계값 [m/s]
  bool zero_external_yaw_on_reset_;   // 리셋 시 IMU yaw를 0으로 정렬

  // conversion gain and offset
  double speed_to_erpm_gain_, speed_to_erpm_offset_;
  double steering_to_servo_gain_, steering_to_servo_offset_;
  double wheelbase_;
  bool publish_tf_;

  // odometry state
  double x_, y_, yaw_;
  std_msgs::Float64::ConstPtr last_servo_cmd_;              ///< Last servo position commanded value
  vesc_msgs::VescStateStamped::ConstPtr last_state_;        ///< Last received state message

  // IMU yaw state
  bool has_imu_yaw_;
  double imu_yaw_unbiased_;          // IMU로부터 받은 원시 yaw(-pi~pi)
  double yaw_bias_;                  // odom 기준 정렬용 바이어스
  double imu_yaw_rate_;              // IMU yaw 차분으로 계산한 yaw rate
  ros::Time last_imu_stamp_;

  // ROS services
  ros::Publisher odom_pub_;
  ros::Subscriber vesc_state_sub_;
  ros::Subscriber servo_sub_;
  ros::Subscriber imu_sub_;
  std::shared_ptr<tf::TransformBroadcaster> tf_pub_;
  ros::ServiceServer reset_srv_;

  std::mutex mtx_;

  // ROS callbacks
  void vescStateCallback(const vesc_msgs::VescStateStamped::ConstPtr& state);
  void servoCmdCallback(const std_msgs::Float64::ConstPtr& servo);
  void imuCallback(const sensor_msgs::Imu::ConstPtr& imu);
  bool resetOdom(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);

  // helpers
  static double normalizeAngle(double a);
};

}  // namespace vesc_ackermann

#endif  // VESC_ACKERMANN_VESC_TO_ODOM_H_