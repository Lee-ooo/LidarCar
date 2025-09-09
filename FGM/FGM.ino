#include <RPLidarC1.h>
#include <ESP32Servo.h>
#include <math.h>

const int SERVO_PIN = 10;  // 舵机引脚
const int DIR_PIN   = 46;  // 电机方向
const int MOTOR_PIN = 21;  // 电机速度

RPLidar lidar;
Servo    servo;

int    servo_angle  = 90;
int    servo_offset = 20;
int    last_velocity = 0;
double e_last = 0;
int    count = 0;

double distances[360] = {0};     // 原始距离（m）
const double min_distance = 0.05; // 盲区最小
const double max_distance = 1.0;  // 兴趣区上限

// ===== 安全气泡参数 =====
const double BUBBLE_R = 0.15; 

static inline int iwrap(int deg) {
  deg %= 360; if (deg < 0) deg += 360; return deg;
}

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
  if (IS_OK(lidar.waitPoint())) {
    double distance = lidar.getCurrentPoint().distance / 1000.0;
    int    angle    = lidar.getCurrentPoint().angle;
    bool   startBit = lidar.getCurrentPoint().startBit;

    if (angle >= 0 && angle < 360) {
      angle = 180 - angle;
      if (angle < 0) angle += 360;
      distances[angle] = distance;
    }
    count++;

    if (startBit) {
      // ========= 安全气泡预处理：生成“膨胀后”的距离表 =========
      static double dist_grow[360];
      // 初始为原始距离（若无效则设较大值，便于 min 更新）
      for (int i = 0; i < 360; ++i) {
        double d = distances[i];
        dist_grow[i] = d - BUBBLE_R;
      }
      // // 对每个有效点做角向膨胀：把邻域角度内的“允许距离”缩短到 d - r
      // for (int i = 0; i < 360; ++i) {
      //   double d = distances[i];
      //   if (d > min_distance && d < max_distance) {
      //     double newd = d - BUBBLE_R;                 // 收缩自由空间
      //     if (newd < min_distance) newd = min_distance;
      //     // 角宽：Δθ ≈ atan(r/d)
      //     int half_w = (int)ceil( atan(BUBBLE_R / d) * 180.0 / M_PI ); // 单位：度
      //     // 覆盖 i-half_w ... i+half_w
      //     for (int k = -half_w; k <= half_w; ++k) {
      //       int j = iwrap(i + k);
      //       // 收缩：可通距离取更小值
      //       if (newd < dist_grow[j]) dist_grow[j] = newd;
      //     }
      //   }
      // }

      // ========== 以下全部用 dist_grow[] 参与计算 ==========
      double front_dist = 0; int c_f = 0;
      for (int i = 170; i <= 190; ++i) { front_dist += dist_grow[i]; ++c_f; }
      front_dist /= (c_f ? c_f : 1);

      double left_dist = 0; int c_l = 0;
      for (int i = 240; i <= 270; ++i) { left_dist += dist_grow[i]; ++c_l; }
      left_dist /= (c_l ? c_l : 1);

      double right_dist = 0; int c_r = 0;
      for (int i =  90; i <= 120; ++i) { right_dist += dist_grow[i]; ++c_r; }
      right_dist /= (c_r ? c_r : 1);

      if (count >= 50) {
        double x1=0,y1=0, x2=0,y2=0;
        double max_spacing = 0;
        double xc=0, yc=0;
        bool flag = false;

        for (int angle_deg = 45; angle_deg < 315; ++angle_deg) {
          double d = dist_grow[angle_deg];
          if (d > min_distance && d < max_distance) {
            double angle_rad = angle_deg * M_PI / 180.0;
            x1 = x2; y1 = y2;
            x2 = cos(angle_rad) * d;
            y2 = sin(angle_rad) * d;

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

        double theta = atan2(yc, xc);
        if (theta < 0) theta += 2.0 * M_PI;

        double e  = theta - M_PI;
        e = atan2(sin(e), cos(e));
        double de = e - e_last;
        e_last = e;

        int deg = int(60 * e + 5 * de);

        // 小角度放大/大角度抑制（保持你的分段系数）
        switch (abs(deg) / 20) {
          case 0: deg = int(deg * 1.35); break;
          case 1: /* 原样 */             break;
          case 2: deg = int(deg * 0.9);  break;
          default: break;
        }
        deg = (int)constrain(deg, -70, 70);

        // 速度：基础 +（可选：前向安全降速）
        int velocity = 160;

        int d_vel = last_velocity - velocity;
        int vel   = velocity + int(0.3 * double(d_vel));
        vel       = (vel > 255) ? 255 : vel;
        if (vel < 0) vel = 0;

        // 输出
        servo.write(servo_angle + servo_offset + deg);
        analogWrite(MOTOR_PIN, vel);
        last_velocity = velocity;
        count = 0;
      }
    }
  } else {
    // 雷达错误，重启
    lidar.startScan();
  }
}
