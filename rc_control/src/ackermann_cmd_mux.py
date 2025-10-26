#!/usr/bin/env python
# -*- coding: utf-8 -*-
import math, time
import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import UInt8, Float32

MODE_ESTOP, MODE_MANUAL, MODE_AUTO = 0, 1, 2

class AckermannMux:
    def __init__(self):
        # 파라미터
        self.timeout = float(rospy.get_param('~command_timeout', 0.2))
        self.input_timeout    = float(rospy.get_param('~input_timeout', 0.2))
        self.steer_max_deg    = float(rospy.get_param('~steer_max_deg', 45.0))
        self.speed_cms_to_mps = float(rospy.get_param('~speed_cms_to_mps', 0.01))
        self.frame_id         = rospy.get_param('~frame_id', 'base_link')

        # 수동(RC) 속도 후처리 파라미터
        self.rc_speed_deadband_cms  = float(rospy.get_param('~rc_speed_deadband_cms', 5.0))     # |cm/s| < deadband -> 0
        self.rc_speed_gain          = float(rospy.get_param('~rc_speed_gain', 1.0))             # 스케일 배율
        self.rc_speed_abs_max_cms   = float(rospy.get_param('~rc_speed_abs_max_cms', 200.0))    # 최종 clamp

        # 입력 캐시
        self.last_manual = None
        self.last_auto   = None
        self.t_manual = 0.0
        self.t_auto   = 0.0

        # RC 원시 입력 값
        self._estop_send = 1   # 기본 정지
        self._auto_send  = 0   # 기본 메뉴얼
        self._speed_cms  = 0.0
        self._steer_deg  = 0.0
        self.t_rc = 0.0

        # 퍼블리셔/서브스크라이버
        self.pub_selected = rospy.Publisher('selected', AckermannDriveStamped, queue_size=10)
        rospy.Subscriber('manual', AckermannDriveStamped, self.cb_manual)
        rospy.Subscriber('auto',   AckermannDriveStamped, self.cb_auto)

        # 아두이노 RC 입력
        rospy.Subscriber('/estop_send', UInt8,   self.cb_estop_send)
        rospy.Subscriber('/auto_send',  UInt8,   self.cb_auto_send)
        rospy.Subscriber('/speed_send', Float32, self.cb_speed_send)
        rospy.Subscriber('/steer_send', Float32, self.cb_steer_send)

        self.rate = rospy.Rate(100)
        self.spin()

    # 콜백
    def cb_manual(self, msg): self.last_manual = msg; self.t_manual = time.time()
    def cb_auto(self, msg):   self.last_auto   = msg; self.t_auto   = time.time()

    def cb_estop_send(self, m): self._estop_send = int(m.data); self.t_rc = time.time()
    def cb_auto_send(self,  m): self._auto_send  = int(m.data); self.t_rc = time.time()
    def cb_speed_send(self, m): self._speed_cms  = float(m.data); self.t_rc = time.time()
    def cb_steer_send(self, m): self._steer_deg  = float(m.data); self.t_rc = time.time()

    # 유틸
    def zero_cmd(self):
        msg = AckermannDriveStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.frame_id
        msg.drive.speed = 0.0
        msg.drive.steering_angle = 0.0
        return msg

    def make_ackermann(self, v_mps, steer_deg):
        sd = max(-self.steer_max_deg, min(self.steer_max_deg, steer_deg))
        msg = AckermannDriveStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.frame_id
        msg.drive.speed = float(v_mps)
        msg.drive.steering_angle = math.radians(sd)
        return msg

    # 메인 루프
    def spin(self):
        while not rospy.is_shutdown():
            now = time.time()
            valid_rc = (now - self.t_rc) < self.input_timeout

            # RC 입력으로부터 모드/에스톱 결정
            estop = (self._estop_send == 1)
            mode = MODE_ESTOP if estop else (MODE_AUTO if self._auto_send == 1 else MODE_MANUAL)

            # 수동 모드 + 유효 RC 입력이면 내부 manual 명령 생성
            if valid_rc and (mode == MODE_MANUAL) and not estop:
                sp_cms = float(self._speed_cms)

                # 1) deadband: |speed| < deadband => 0
                if abs(sp_cms) < self.rc_speed_deadband_cms:
                    sp_cms = 0.0

                # 2) gain 적용
                sp_cms *= self.rc_speed_gain

                # 3) 최종 클램프
                if sp_cms > self.rc_speed_abs_max_cms:
                    sp_cms = self.rc_speed_abs_max_cms
                elif sp_cms < -self.rc_speed_abs_max_cms:
                    sp_cms = -self.rc_speed_abs_max_cms

                # 4) m/s 변환
                v_mps = sp_cms * self.speed_cms_to_mps

                manual_msg = self.make_ackermann(v_mps, self._steer_deg)
                self.last_manual = manual_msg
                self.t_manual = now

            # 선택 로직
            cmd = self.zero_cmd()
            if estop or mode == MODE_ESTOP:
                cmd = self.zero_cmd()
            elif mode == MODE_MANUAL:
                if self.last_manual and (now - self.t_manual) < self.timeout:
                    cmd = self.last_manual
            elif mode == MODE_AUTO:
                if self.last_auto and (now - self.t_auto) < self.timeout:
                    cmd = self.last_auto

            cmd.header.stamp = rospy.Time.now()
            self.pub_selected.publish(cmd)
            self.rate.sleep()

if __name__ == '__main__':
    rospy.init_node('ackermann_cmd_mux')
    AckermannMux()