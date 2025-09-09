#include <RPLidarC1.h>
#include <ESP32Servo.h>
#include <math.h>

const int SERVO_PIN = 10;  // 舵机引脚
const int DIR_PIN = 46;    // 电机方向
const int MOTOR_PIN = 21;  // 电机速度

RPLidar lidar;  // 创建激光雷达对象
Servo servo;    // 创建舵机对象

int servo_angle = 90;
int servo_offset = 20;  // 用于调整舵机安装误差
int deg = -50;
int i = 0;

void setup() {
  Serial.begin(115200);
  lidar.begin(Serial2);
  lidar.startScan();
  servo.attach(SERVO_PIN);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(MOTOR_PIN, OUTPUT);
  digitalWrite(DIR_PIN, HIGH);
  servo.write(servo_angle + servo_offset);
  analogWrite(MOTOR_PIN, 150);
}

void loop() {
  i++;
  i %= 45;
  servo.write(servo_angle + servo_offset + i);
  delay(500);
}
