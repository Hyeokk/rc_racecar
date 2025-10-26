#include <ros.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Float32.h>

// ===== 통신 속도(양쪽 동일하게 설정) =====
#define BAUD 57600  // 57600, 115200, 230400, 250000 등

// 핀 정의
const unsigned int SPEED_PIN = 4;   // 스로틀 PWM
const unsigned int STEER_PIN = 5;   // 조향 PWM
const unsigned int MODE_PIN = 6;   // 주행/중립/후진 (1000,1500,2000)
const unsigned int ESTOP_PIN = 7;   // E-Stop (<=1500 주행가능, >1500 정지)
const unsigned int AUTO_PIN = 8;   // Auto (<=1500 Manual, >=1500 Auto)

const unsigned int MIDDLE_STEER = 1480;
const unsigned int MIDDLE_SPEED = 1480;
const unsigned int PWM_MAX = 2000;
const unsigned int PWM_MIN = 1000;

// 퍼블리시 변수
unsigned int is_auto   = 0;   // 0: manual, 1: auto
unsigned int is_auto_raw   = 0;
unsigned int is_estop  = 0;   // 0: run,    1: stop
unsigned int is_estop_raw  = 0;
int speed_data = 0;   // 전/후진 포함: -200 ~ +200   <-- 변경됨(후진 시 음수)
int steer_data = 0;   // -45 ~ +45 (deg)
int mode_raw  = 0;   // -1: 후진, 0: 중립, 1: 주행
int mode_out = 1;

std_msgs::UInt8 auto_msg;
std_msgs::UInt8 estop_msg;
std_msgs::Float32 speed_msg;
std_msgs::Float32 steer_msg;
std_msgs::UInt8 mode_msg;

ros::NodeHandle nh;
ros::Publisher pub_auto ("auto_send",  &auto_msg);
ros::Publisher pub_estop("estop_send", &estop_msg);
ros::Publisher pub_speed("speed_send", &speed_msg);
ros::Publisher pub_steer("steer_send", &steer_msg);
ros::Publisher pub_mode ("mode_send",  &mode_msg);

void setup() {
  Serial.begin(BAUD);

  pinMode(SPEED_PIN, INPUT);
  pinMode(STEER_PIN, INPUT);
  pinMode(MODE_PIN, INPUT);
  pinMode(ESTOP_PIN, INPUT);
  pinMode(AUTO_PIN, INPUT);

  nh.initNode();
  nh.advertise(pub_speed);
  nh.advertise(pub_steer);
  nh.advertise(pub_mode);
  nh.advertise(pub_estop);
  nh.advertise(pub_auto);
}

void loop() {
  // 1) PWM 읽기
  speed_data = pulseIn(SPEED_PIN, HIGH); // throttle (1000~2000)
  steer_data = pulseIn(STEER_PIN, HIGH); // steer
  mode_raw = pulseIn(MODE_PIN, HIGH); // mode
  is_estop_raw = pulseIn(ESTOP_PIN, HIGH); // estop
  is_auto_raw = pulseIn(AUTO_PIN, HIGH); // auto

  // SPEED: 1000~2000 -> 0~200
  long t_mag = map(speed_data, PWM_MIN, PWM_MAX, 0, 200);
  if (t_mag < 0)   t_mag = 0;
  if (t_mag > 200) t_mag = 200;
  int speed_out = (int)t_mag;   // 후진 모드면 아래에서 음수로 전환

  // STEER: 1000~2000 -> -45~+45
  long s = map(steer_data, PWM_MIN, PWM_MAX, -45, 45);
  // 필요 시 방향 반전: map(ch2, PWM_MIN, PWM_MAX, 45, -45);
  if (s < -45) s = -45;
  if (s >  45) s = 45;
  int steer_out = (int)s;

  // CH3: 모드(≤1400: 주행(전진), 1400~1600: 중립, ≥1600: 후진)
  if (mode_raw > 1400 && mode_raw < 1600)      mode_out = 0;   // 중립
  else if (mode_raw <= 1400)              mode_out = 1;   // 주행(전진)
  else                               mode_out = -1;  // 후진

  // CH4: E-Stop (<=1500: 주행가능 0,  >1500: 정지 1)
  is_estop = (is_estop_raw > 1500) ? 1 : 0;
  // CH5: Auto  (<=1500: Manual 0,   >=1500: Auto 1)
  is_auto = (is_auto_raw >= 1500) ? 1 : 0;

  // 3) 속도 부호 결정: 후진 모드면 음수
  speed_out = (int)t_mag;
  if (mode_out == -1) {
    speed_out = -speed_out;  // 후진 시 음수로 전송
  }
  if (mode_out == 0) {
    speed_out = 0;
  }

  // 4) 퍼블리시
  auto_msg.data  = is_auto;
  estop_msg.data = is_estop;
  speed_msg.data = speed_out;
  steer_msg.data = steer_out;
  mode_msg.data  = mode_out;

  pub_auto.publish(&auto_msg);
  pub_estop.publish(&estop_msg);
  pub_speed.publish(&speed_msg);
  pub_steer.publish(&steer_msg);
  pub_mode.publish(&mode_msg);

  nh.spinOnce();

  // 디버그 출력
  Serial.println(F("=============================="));
  Serial.print(F("CH1 throttle(us): ")); Serial.println(speed_data);
  Serial.print(F("CH2 steer(us):    ")); Serial.println(steer_data);
  Serial.print(F("CH3 mode(us):     ")); Serial.println(mode_raw);
  Serial.print(F("CH4 estop(us):    ")); Serial.println(is_estop);
  Serial.print(F("CH5 auto(us):     ")); Serial.println(is_auto);

  Serial.print(F("Speed_out (-200~200): ")); Serial.println(speed_out);
  Serial.print(F("Steer_out (deg):      ")); Serial.println(steer_out);
  Serial.print(F("Estop:                 ")); Serial.println(is_estop);
  Serial.print(F("Auto:                  ")); Serial.println(is_auto);
  Serial.print(F("Mode:                  ")); Serial.println(mode_out);

  delay(10); // ~100 Hz
}
