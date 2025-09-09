#include <RPLidarC1.h>
#include <ESP32Servo.h>
#include <math.h>

const int SERVO_PIN = 10;  // 舵机引脚
const int DIR_PIN = 46;    // 电机方向
const int MOTOR_PIN = 21; // 电机速度

RPLidar lidar;  // 创建激光雷达对象
Servo servo;    // 创建舵机对象

int servo_angle = 90;
int servo_offset = 18;  // 用于调整舵机安装误差
int last_velocity = 0;
double e_last = 0;  // PID 误差缓存
int count = 0;      // 对每一圈扫描点个数进行计数

double distances[360] = { 0 };     // 定义数组 存储激光雷达数据
const double min_distance = 0.05;  // 雷达数据兴趣区间 最小值(盲区)
const double max_distance = 1;   // 雷达数据兴趣区间 最大值
// int c = 0;

void setup() {
  Serial.begin(115200);                     // 设置调试串口
  lidar.begin(Serial2);                     // 设置激光雷达串口
  lidar.startScan();                        // 启动雷达扫描
  servo.attach(SERVO_PIN);                  // 连接舵机引脚
  servo.write(servo_angle + servo_offset);  // 舵机初始值

  pinMode(DIR_PIN, OUTPUT);     //电机方向
  pinMode(MOTOR_PIN, OUTPUT);   //电机速度
  digitalWrite(DIR_PIN, HIGH);  //HIGH 电机正转 LOW 电机反转
  delay(1000);
}

void loop() {
  if (IS_OK(lidar.waitPoint())) {                               // 等到一个新的扫描点
    double distance = lidar.getCurrentPoint().distance / 1000.0;  // 距离值，单位m
    int angle = lidar.getCurrentPoint().angle;                  // 角度值（整数，四舍五入）
    bool startBit = lidar.getCurrentPoint().startBit;           // 进入新的一圈扫描的标志位
    // Serial.printf("%d, %f\r\n", angle, distance);            // 串口观察数据 正前方为0度，顺时针
    if (angle >= 0 && angle < 360) {  // 雷达数据转换 正后方启动 逆时针数据
      angle = 180 - angle;
      if (angle < 0) {
        angle += 360;
      }
      distances[angle] = distance;
    }
    count++;

    if (startBit) {  // 每进入一次新的扫描处理并控制一次，雷达数据刷新频率约10Hz
      double front_dist = 0;
      int c_f = 0;
      for(int i = 170;i <= 190;i++)
      {
        front_dist += distances[i];
        c_f++;
      }
      front_dist /= c_f;

      double xl = 0, yl = 0;
      xl = cos(240*M_PI/180) * distances[240] - cos(270*M_PI/180) * distances[270];
      yl = sin(240*M_PI/180) * distances[240] - sin(270*M_PI/180) * distances[270];

      double xr = 0, yr = 0;
      xr = cos(120*M_PI/180) * distances[120] - cos(90*M_PI/180) * distances[90];
      yr = sin(120*M_PI/180) * distances[120] - sin(90*M_PI/180) * distances[90];

      double x_imp = (xl + xr)/2;
      double y_imp = (yl + yr)/2;

      double left_dist = 0;
      int c_l = 0;
      for (int i = 240; i <= 270; i++) { left_dist += distances[i]; c_l++; }
      left_dist /= c_l;

      double right_dist = 0;
      int c_r = 0;
      for (int i = 90; i <= 150; i++) { right_dist += distances[i]; c_r++; }
      right_dist /= c_r;

      if (count >= 50) {
        double x1, y1 = 0;       // 前一个扫描点位置
        double x2, y2 = 0;       // 当前扫描点位置
        double max_spacing = 0;  // 相邻扫描点之间的最大距离
        double xc, yc;           // 赛道中点

        bool flag = false;       // 标志位

        for (int angle_deg = 45; angle_deg < 315; angle_deg++) {
          distance = distances[angle_deg];
          if (distance > min_distance && distance < max_distance) {  // 仅处理雷达数据兴趣区间内的数据
            double angle_rad = (double)angle_deg / 180.0 * M_PI;     // 角度转弧度

            // 将上个循环的当前扫描点赋给前一个扫描点
            x1 = x2;
            y1 = y2;
            // 计算当前扫描点位置
            x2 = cos(angle_rad) * distance;
            y2 = sin(angle_rad) * distance;

            if (flag) {
              // 计算相邻扫描点之间的距离
              double spacing = sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
              if (spacing > max_spacing) {
                // 更新相邻扫描点之间的最大距离
                max_spacing = spacing;
                // 更新赛道中点
                xc = (x1 + x2) / 2;
                yc = (y1 + y2) / 2;
              }
            }
            flag = true;
          }
        }
        double xm = 0, ym = 0;
        xm = xc + x_imp;
        ym = yc + y_imp;
        // Serial.printf("%f ", atan2(y_imp, x_imp)/M_PI*180);
        double theta = atan2(ym, xm);  // 计算期望朝向
        double lr_imp = 13*log(left_dist/right_dist+1e-7)/180*M_PI;
        // Serial.printf("lr_imp:%f ", lr_imp*180/M_PI);
        // Serial.printf("%f ", theta/M_PI*180-180);
        theta += lr_imp;//左右比值修正
        if(theta < 0) theta += 2*M_PI;
        // Serial.printf("%f ", theta/M_PI*180-180);
        double e = theta - M_PI;       // 计算期望朝向与实际朝向(pi)的误差
        // Serial.printf("%f ", e/M_PI*180);
        e = atan2(sin(e), cos(e));     // 将误差角度统一到[-pi, pi]范围
        double de = e - e_last;               // 误差增量
        e_last = e;                           // 缓存误差数据，用于下一次计算误差增量
        int deg = 70 * e + 5 * de;  // PD控制舵机角度
        deg *= (abs(deg) < 20)?1.35:1;
        // int velocity = 160 + 40 * exp(-1*double(M_PI-abs(theta))*double(M_PI-abs(theta))*8.2);
        int velocity = 143;
        int d_vel = last_velocity - velocity;
        int vel = velocity + 0.2 * double(d_vel);
        // double slow_dist = 0.6;
        // if(front_dist <= slow_dist)
        // {
        //   deg *= 1.25;
        //   vel *= constrain(front_dist/slow_dist, 0.7, 1);
        // }
        deg = int(constrain(deg, -55, 55));
        // Serial.printf("%d ",deg);
        vel = (vel > 255)?255:vel;
        // deg = constrain(c-45,-45,45);//阻塞检查
        // c++;
        // c %= 90;
        servo.write(servo_angle + servo_offset + deg);
        analogWrite(MOTOR_PIN, vel);
        last_velocity = velocity;
        count = 0;
        // Serial.println();
      }
    }
  } else {  // 出现错误，重新启动雷达
    lidar.startScan();
  }
}