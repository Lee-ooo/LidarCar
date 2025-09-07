#include <RPLidarC1.h>
#include <ESP32Servo.h>
#include <math.h>

const int SERVO_PIN = 10;  // 舵机引脚
const int DIR_PIN = 46;    // 电机方向
const int MOTOR_PIN = 21;  // 电机速度

RPLidar lidar;  // 创建激光雷达对象
Servo servo;    // 创建舵机对象

int servo_angle = 90;
int servo_offset = 13;  // 用于调整舵机安装误差
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
  servo.write(servo_angle + servo_offset);

  pinMode(DIR_PIN, OUTPUT);
  pinMode(MOTOR_PIN, OUTPUT);
  digitalWrite(DIR_PIN, HIGH);
  delay(1000);
}

void loop() {
  if (IS_OK(lidar.waitPoint())) {
    double distance = lidar.getCurrentPoint().distance / 1000.0;  // m
    int angle = lidar.getCurrentPoint().angle;
    bool startBit = lidar.getCurrentPoint().startBit;

    if (angle >= 0 && angle < 360) {  // 正后方启动 逆时针数据
      angle = 180 - angle;
      if (angle < 0) angle += 360;
      distances[angle] = distance;
    }
    count++;

    if (startBit) {
      max_distance = 1;



      double front_dist = 0;
      int c_f = 0;
      for (int i = 170; i <= 190; i++) { front_dist += distances[i]; c_f++; }
      front_dist /= c_f;

      if (count >= 50) {
        double x1 = 0, y1 = 0, x2 = 0, y2 = 0;
        double max_spacing = -1;
        double xc = 0, yc = 0;
        double d1 = 0, d2 = 0, dist1 = 0, dist2 = 0;
        bool flag = false;
        // if(last_velocity > 0) max_distance = constrain(double(last_velocity)/210.0, 0.8, 1.2);

        // ★ ND: 记录“最大缝隙端点”的角度，供邻域差分用
        double bestLx = 0, bestLy = 0; int bestLa = -1;
        double bestRx = 0, bestRy = 0; int bestRa = -1;

        for(int i = 0; i < 2; i++){
          Serial.print(max_distance);
          Serial.print(" ");
          x1 = 0, x2 = 0, y1 = 0, y2 = 0, xc = 0, yc = 0, d1 = 0, d2 = 0, dist1 = 0, dist2 = 0, max_spacing = -1, flag = false;
          for (int angle_deg = 45; angle_deg < 315; angle_deg++) {
            distance = distances[angle_deg];
            if (distance > min_distance && distance < max_distance) {
              double angle_rad = (double)angle_deg * M_PI / 180.0;

              x1 = x2; y1 = y2; d1 = d2;
              d2 = distance;
              x2 = cos(angle_rad) * distance;
              y2 = sin(angle_rad) * distance;

              if (flag) {
                double dx = x1 - x2, dy = y1 - y2;
                double spacing = sqrt(dx * dx + dy * dy);
                if (spacing > max_spacing) {
                  max_spacing = spacing;
                  dist1 = d1; dist2 = d2;
                  xc = (x1 + x2) * 0.5;
                  yc = (y1 + y2) * 0.5;
                  bestLx = x1; bestLy = y1; bestLa = angle_deg - 1; // 上一个有效点的角
                  bestRx = x2; bestRy = y2; bestRa = angle_deg;     // 当前有效点的角
                }
              }
              flag = true;
            }
          }
          max_distance = constrain(min(dist1, dist2), 0.8, 1);
        }
        // ========== 邻域差分法切线偏置 ========== //
        double theta;
        if (max_spacing < 0) {
          theta = M_PI;  // 兜底：直行
        } else {
          // 中点方向
          double vmx = xc, vmy = yc; normalize2(vmx, vmy);

          // ★ ND: 邻域差分法参数
          const int DELTA = 3;     // 两侧邻域半径（度）：取 a-DELTA 到 a-1 和 a+1 到 a+DELTA
          const int AVG_W = 0;     // 每侧再扩一点做平均，0表示不扩；可设为 1~2 提升稳健性
          auto sampleAvgPoint = [&](int a0, int da_from, int da_to, double &ox, double &oy)->bool{
            // 在角度区间 [a0+da_from, a0+da_to] 取平均点坐标
            double sx=0, sy=0; int cnt=0;
            for(int da=da_from; da<=da_to; ++da){
              int a = wrapDeg(a0 + da);
              double r = distances[a];
              if (r > min_distance && r < max_distance) {
                double ar = a * M_PI / 180.0;
                sx += cos(ar) * r; sy += sin(ar) * r; cnt++;
              }
            }
            if(cnt==0) return false;
            ox = sx / cnt; oy = sy / cnt; return true;
          };

          auto tangentByND = [&](int a_endpoint, double px, double py, double &tx, double &ty)->bool{
            if (a_endpoint < 0) return false;
            // 左侧邻域（减小角度）
            double lm_x=0, lm_y=0, lp_x=0, lp_y=0;
            bool ok_minus = sampleAvgPoint(a_endpoint, -(DELTA+AVG_W), -1, lm_x, lm_y);
            // 右侧邻域（增大角度）
            bool ok_plus  = sampleAvgPoint(a_endpoint, 1, (DELTA+AVG_W), lp_x, lp_y);
            if (ok_minus && ok_plus) {
              tx = lp_x - lm_x; ty = lp_y - lm_y;
              double s = sqrt(tx*tx + ty*ty);
              if (s < 1e-6) { tx = 0; ty = 0; return false; }
              tx /= s; ty /= s;
              // 让切线指向通道内部：与 v_mid 点积为负则翻转
              if (tx*vmx + ty*vmy < 0) { tx = -tx; ty = -ty; }
              return true;
            }
            // 邻域不足：退回到简化 rot90 法
            double nx = px, ny = py; normalize2(nx, ny);
            tx = -ny; ty = nx;
            if (tx*vmx + ty*vmy < 0) { tx = -tx; ty = -ty; }
            return true;
          };

          double t1x=0, t1y=0, t2x=0, t2y=0;
          bool ok1 = tangentByND(bestLa, bestLx, bestLy, t1x, t1y); // 左端点切线（邻域差分）
          bool ok2 = tangentByND(bestRa, bestRx, bestRy, t2x, t2y); // 右端点切线（邻域差分)

          double tx = t1x + t2x, ty = t1y + t2y;
          if (fabs(tx) < 1e-9 && fabs(ty) < 1e-9) { // 极端兜底：用中点方向
            tx = vmx; ty = vmy;
          } else {
            normalize2(tx, ty);
          }

          const double ALPHA = 1;  // 中线/切线融合权重
          double vx = ALPHA * vmx + (1.0 - ALPHA) * tx;
          double vy = ALPHA * vmy + (1.0 - ALPHA) * ty;
          theta = atan2(vy, vx);
        }

        double lr_imp = 5*log(left_dist/right_dist+1e-7);
        Serial.print("lr_imp:");
        Serial.print(lr_imp);
        Serial.print(" ");
        theta += lr_imp;//左右比值修正

        // ====== 邻域差分法切线偏置结束 ====== //
        if(theta < 0) theta += 2*M_PI;
        Serial.print(theta/M_PI*180 - 180);
        Serial.print(" ");
        double e = theta - M_PI;
        e = atan2(sin(e), cos(e));
        double de = e - e_last;
        e_last = e;
        servo_angle = 55 * e + 5 * de;
        int deg = servo_angle + servo_offset;
        Serial.println(deg);

        int velocity = 150 + 30 * exp(-1 * double(M_PI - abs(theta)) * double(M_PI - abs(theta)) * 8.2);
        int d_vel = last_velocity - velocity;
        int vel = velocity + 0.2 * double(d_vel);

        double slow_dist = 0.6;
        if (front_dist <= slow_dist) {
          deg *= 1.5;
        }
        deg = constrain(deg, -60, 60);
        vel = (vel > 255) ? 255 : vel;
        // //倒车
        // if(avoid_dist < 0.25){
        //   servo.write(90+servo_offset);
        //   digitalWrite(DIR_PIN, LOW);
        //   analogWrite(MOTOR_PIN, 100);
        // }else{
        // }
        servo.write(90 + deg);
        digitalWrite(DIR_PIN, HIGH);
        analogWrite(MOTOR_PIN, vel);
        last_velocity = velocity;
        count = 0;
      }
    }
  } else {
    lidar.startScan();
  }
}
