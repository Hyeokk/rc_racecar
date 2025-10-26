#!/usr/bin/env python
import rospy, time
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Float64

class Interpolator:
    def __init__(self):
        p = rospy.get_param
        self.max_accel  = float(p('~max_acceleration', 1.5))
        self.max_decel  = float(p('~max_deceleration', 2.5))
        self.max_servo_speed = float(p('~max_servo_speed', 2.5))
        self.th_rate = float(p('~throttle_rate_hz', 40.0))
        self.sv_rate = float(p('~servo_rate_hz', 45.0))
        self.timeout = float(p('~command_timeout', 0.2))

        self.last_in = None
        self.t_last = 0.0
        self.curr_v = 0.0
        self.curr_delta = 0.0

        # 단위 변환(ackermann_to_vesc와 동일 계열)
        self.g_v  = float(p('~speed_to_erpm_gain', 4614.0))
        self.o_v  = float(p('~speed_to_erpm_offset', 0.0))
        self.g_sv = float(p('~steering_angle_to_servo_gain', 0.6))
        self.o_sv = float(p('~steering_angle_to_servo_offset', 0.5))

        # VESC 직접 퍼블리시 스위치
        self.direct_vesc = bool(p('~direct_vesc', False))

        self.pub_cmd = rospy.Publisher('cmd_out', AckermannDriveStamped, queue_size=10)
        self.pub_speed_erpm = rospy.Publisher('/vesc/commands/motor/speed', Float64, queue_size=10)
        self.pub_servo = rospy.Publisher('/vesc/commands/servo/position', Float64, queue_size=10)

        rospy.Subscriber('cmd_in', AckermannDriveStamped, self.cb_in)

        self.rate  = rospy.Rate(int(max(self.th_rate, self.sv_rate)))
        self.loop()

    def cb_in(self, msg):
        self.last_in = msg
        self.t_last = time.time()

    def step_limit(self, target, current, max_rate, dt):
        diff = target - current
        lim  = max_rate * dt
        if diff >  lim: return current + lim
        if diff < -lim: return current - lim
        return target

    def loop(self):
        while not rospy.is_shutdown():
            now = time.time()

            valid = self.last_in and (now - self.t_last) < self.timeout

            if not valid:
                self.curr_v     = 0.0
                self.curr_delta = 0.0
                out = AckermannDriveStamped()
                out.header.stamp = rospy.Time.now()
                out.drive.speed = 0.0
                out.drive.steering_angle = 0.0
                self.pub_cmd.publish(out)
                if self.direct_vesc:
                    self.pub_speed_erpm.publish(Float64(self.g_v * 0.0 + self.o_v))
                    self.pub_servo.publish(Float64(self.g_sv * 0.0 + self.o_sv))
                self.rate.sleep()
                continue

            target_v     = float(self.last_in.drive.speed)
            target_delta = float(self.last_in.drive.steering_angle)

            max_rate_v = self.max_accel if target_v > self.curr_v else self.max_decel
            self.curr_v = self.step_limit(target_v, self.curr_v, max_rate_v, 1.0/self.th_rate)
            self.curr_delta = self.step_limit(target_delta, self.curr_delta, self.max_servo_speed, 1.0/self.sv_rate)

            out = AckermannDriveStamped()
            out.header.stamp = rospy.Time.now()
            out.drive.speed = self.curr_v
            out.drive.steering_angle = self.curr_delta
            self.pub_cmd.publish(out)

            if self.direct_vesc:
                self.pub_speed_erpm.publish(Float64(self.g_v * self.curr_v + self.o_v))
                self.pub_servo.publish(Float64(self.g_sv * self.curr_delta + self.o_sv))

            self.rate.sleep()

if __name__ == '__main__':
    rospy.init_node('throttle_interpolator')
    Interpolator()