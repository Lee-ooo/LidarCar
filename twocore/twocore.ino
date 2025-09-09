#include <Arduino.h>
#include <RPLidarC1.h>
#include <ESP32Servo.h>
#include <math.h>

// ================== Pin 配置（按你现有为准，若S3连线不同请改） ==================
const int SERVO_PIN = 10;   // 舵机引脚（LEDC PWM）
const int DIR_PIN   = 46;   // 电机方向
const int MOTOR_PIN = 21;   // 电机速度（LEDC PWM）

// 如需自定义S3串口引脚，取消注释并设置：
// #define RX2_PIN  5
// #define TX2_PIN  6

// ================== 全局对象 ==================
RPLidar lidar;
Servo    servo;

// ================== 控制参数&状态 ==================
int    servo_angle  = 90;
int    servo_offset = 18;      // 舵机安装偏置
int    last_velocity = 0;
double e_last = 0;             // PD缓存
int    count  = 0;             // 本圈接收计数

// 雷达参数
double distances[360]      = {0};  // 采集中工作缓冲区
double frameDistances[360] = {0};  // 完整一圈的快照供控制任务使用
const double min_distance = 0.05;
const double max_distance = 1.0;

// ================== RTOS 同步原语 ==================
TaskHandle_t lidarTaskHandle   = nullptr;
TaskHandle_t controlTaskHandle = nullptr;
SemaphoreHandle_t frameMutex   = nullptr;  // 保护 frameDistances 的互斥锁

// 让采集任务在每次新圈准备好后“叫醒”控制任务
static inline void notifyControlTask() {
  if (controlTaskHandle) {
    xTaskNotifyGive(controlTaskHandle);
  }
}

// ================== 工具函数 ==================
static inline int iwrap(int deg) {
  // 将角度索引约束到[0,359]
  deg %= 360;
  if (deg < 0) deg += 360;
  return deg;
}

// ================== 任务：Core 0 雷达采集 ==================
void LidarTask(void* pv) {
    // 串口与雷达启动
  #ifdef RX2_PIN
    Serial2.begin(115200, SERIAL_8N1, RX2_PIN, TX2_PIN);
    lidar.begin(Serial2);
  #else
    lidar.begin(Serial2);
  #endif
    lidar.startScan();

  for (;;) {
    if (IS_OK(lidar.waitPoint())) {
      double distance = lidar.getCurrentPoint().distance / 1000.0;  // m
      int    angle    = lidar.getCurrentPoint().angle;              // 0..359
      bool   startBit = lidar.getCurrentPoint().startBit;

      if (angle >= 0 && angle < 360) {
        // 你的坐标系变换：正后方启动，逆时针数据
        int mapped = 180 - angle;
        if (mapped < 0) mapped += 360;
        distances[mapped] = distance;
      }
      count++;

      // 到达新的一圈：复制快照 -> 通知控制任务处理
      if (startBit) {
        if (frameMutex && xSemaphoreTake(frameMutex, portMAX_DELAY) == pdTRUE) {
          memcpy(frameDistances, distances, sizeof(frameDistances));
          xSemaphoreGive(frameMutex);
        }
        // 你原逻辑用 count>=50 才做一次控制，这里把条件留给控制任务；
        // 采集任务总是先快照后通知，控制任务自行决定是否处理。
        notifyControlTask();
        count = 0;
      }
    } else {
      // 雷达异常，重启扫描
      lidar.startScan();
      vTaskDelay(pdMS_TO_TICKS(5));
    }

    // 给其它任务让点时间片
    taskYIELD();
  }
}

// ================== 任务：Core 1 控制与执行 ==================
void ControlTask(void* pv) {
  for (;;) {
    // 等待采集任务的“新圈就绪”通知
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    // 取一圈快照
    double local[360];
    if (frameMutex && xSemaphoreTake(frameMutex, portMAX_DELAY) == pdTRUE) {
      memcpy(local, frameDistances, sizeof(local));
      xSemaphoreGive(frameMutex);
    } else {
      continue; // 取不到就跳过本轮
    }

    // ============ 以下基本保持你原有的每圈处理逻辑 ============
    // front_dist（170..190）
    double front_dist = 0; int c_f = 0;
    for (int i = 170; i <= 190; ++i) { front_dist += local[iwrap(i)]; ++c_f; }
    front_dist /= (c_f ? c_f : 1);

    // 左右向量割线（240/270 与 120/90）
    double xl = cos(240*M_PI/180.0) * local[240] - cos(270*M_PI/180.0) * local[270];
    double yl = sin(240*M_PI/180.0) * local[240] - sin(270*M_PI/180.0) * local[270];
    double xr = cos(120*M_PI/180.0) * local[120] - cos( 90*M_PI/180.0) * local[ 90];
    double yr = sin(120*M_PI/180.0) * local[120] - sin( 90*M_PI/180.0) * local[ 90];

    double x_imp = (xl + xr) / 2.0;
    double y_imp = (yl + yr) / 2.0;

    // 左右距离均值
    double left_dist = 0;  int c_l = 0;
    for (int i = 240; i <= 270; ++i) { left_dist += local[iwrap(i)]; ++c_l; }
    left_dist /= (c_l ? c_l : 1);

    double right_dist = 0; int c_r = 0;
    for (int i =  90; i <= 150; ++i) { right_dist += local[iwrap(i)]; ++c_r; }
    right_dist /= (c_r ? c_r : 1);

    // 最大缝隙与赛道中点
    double x1=0,y1=0,x2=0,y2=0, max_spacing=0, xc=0, yc=0;
    bool flag=false;

    for (int angle_deg = 45; angle_deg < 315; ++angle_deg) {
      double distance = local[angle_deg];
      if (distance > min_distance && distance < max_distance) {
        double angle_rad = angle_deg * M_PI / 180.0;
        x1 = x2; y1 = y2;
        x2 = cos(angle_rad) * distance;
        y2 = sin(angle_rad) * distance;
        if (flag) {
          double dx = x1 - x2, dy = y1 - y2;
          double spacing = sqrt(dx*dx + dy*dy);
          if (spacing > max_spacing) {
            max_spacing = spacing;
            xc = (x1 + x2) / 2.0;
            yc = (y1 + y2) / 2.0;
          }
        }
        flag = true;
      }
    }

    // 融合补偿向量
    double xm = xc + x_imp;
    double ym = yc + y_imp;

    double theta = atan2(ym, xm);  // 期望朝向
    double lr_imp = 16 * log(left_dist / (right_dist + 1e-7) + 1e-7) / 180.0 * M_PI;
    theta += lr_imp;               // 左右比值修正
    if (theta < 0) theta += 2.0 * M_PI;

    double e  = theta - M_PI;
    e = atan2(sin(e), cos(e));     // wrap 到 [-pi, pi]
    double de = e - e_last;
    e_last = e;

    int deg = int(70 * e + 5 * de);
    deg *= (abs(deg) < 20) ? 1.35 : 1.0;
    deg = (int)constrain(deg, -55, 55);

    int velocity = 160 + int(40 * exp(-1 * (M_PI - fabs(theta)) * (M_PI - fabs(theta)) * 8.2));
    int d_vel    = last_velocity - velocity;
    int vel      = velocity + int(0.2 * double(d_vel));
    vel          = (vel > 255) ? 255 : vel;
    if (vel < 0) vel = 0; // 保险

    // 输出执行
    servo.write(servo_angle + servo_offset + deg);
    analogWrite(MOTOR_PIN, vel);
    digitalWrite(DIR_PIN, HIGH); // 正转；若需倒车可切LOW并设置对应PWM策略
    last_velocity = velocity;

    // 可选：调试输出
    // Serial.printf("deg:%d vel:%d theta:%.2f e:%.2f\n", deg, vel, theta*180/M_PI, e*180/M_PI);
  }
}

// ================== Arduino 入口 ==================
void setup() {
  Serial.begin(115200);

  // 舵机&电机
  servo.attach(SERVO_PIN);
  servo.write(servo_angle + servo_offset);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(MOTOR_PIN, OUTPUT);
  digitalWrite(DIR_PIN, HIGH);
  delay(500);

  // 互斥锁
  frameMutex = xSemaphoreCreateMutex();

  // 创建双核任务
  // 采集任务放 core 0，优先级高一点保证串口读不堵
  xTaskCreatePinnedToCore(LidarTask,   "LidarTask",   8192, nullptr, 3, &lidarTaskHandle,   0);
  // 控制任务放 core 1
  xTaskCreatePinnedToCore(ControlTask, "ControlTask", 8192, nullptr, 2, &controlTaskHandle, 1);
}

void loop() {
  // 空着即可，逻辑都在两个任务里
  vTaskDelay(pdMS_TO_TICKS(1000));
}
