#include <RPLidarC1.h>
#include <ESP32Servo.h>
#include <math.h>

const int SERVO_PIN = 10;  // 舵机引脚
const int DIR_PIN = 46;    // 电机方向
const int MOTOR_PIN = 21;  // 电机速度

RPLidar lidar;  // 创建激光雷达对象
Servo servo;    // 创建舵机对象

int servo_angle = 90;
int servo_offset = -3;  // 用于调整舵机安装误差
int last_velocity = 0;
double e_last = 0;  // PID 误差缓存
int count = 0;      // 对每一圈扫描点个数进行计数

double distances[360] = {0};      // 定义数组 存储激光雷达数据
const double min_distance = 0.05; // 雷达数据兴趣区间 最小值(盲区)
double max_distance = 1;    // 雷达数据兴趣区间 最大值
// int c = 0;

// —— 小工具函数 —— //
static inline void normalize2(double &x, double &y) {
  double s = sqrt(x * x + y * y);
  if (s < 1e-9) { x = 0; y = 0; } else { x /= s; y /= s; }
}
static inline int wrapDeg(int a){ a%=360; if(a<0)a+=360; return a; }

void setup() {
  Serial.begin(115200);
  lidar.begin(Serial2);
  lidar.startScan();
  servo.attach(SERVO_PIN);
  servo.write(servo_angle + servo_offset+45);

  pinMode(DIR_PIN, OUTPUT);
  pinMode(MOTOR_PIN, OUTPUT);
  digitalWrite(DIR_PIN, HIGH);
  delay(1000);
}

void loop() {
  servo.write(90 - 3);
  digitalWrite(DIR_PIN, HIGH);
  analogWrite(MOTOR_PIN, 200);
}
