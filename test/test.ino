#include <RPLidarC1.h>
#include <ESP32Servo.h>
#include <math.h>

const int SERVO_PIN = 10;  // 舵机引脚
const int DIR_PIN = 46;    // 电机方向
const int MOTOR_PIN = 21;  // 电机速度

RPLidar lidar;  // 创建激光雷达对象
Servo servo;    // 创建舵机对象

int servo_angle = 90;
int servo_offset = 18;  // 用于调整舵机安装误差
int deg = -20;

void setup() {
  Serial.begin(115200);
  lidar.begin(Serial2);
  lidar.startScan();
  servo.attach(SERVO_PIN);
  servo.write(servo_angle + servo_offset);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(MOTOR_PIN, OUTPUT);
  digitalWrite(DIR_PIN, HIGH);
  delay(1000);
}

void loop() {
  servo.write(servo_angle + servo_offset +deg);
  digitalWrite(DIR_PIN, HIGH);
  analogWrite(MOTOR_PIN, 100);
}
