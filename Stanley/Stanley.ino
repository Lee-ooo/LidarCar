#include <RPLidarC1.h>
#include <ESP32Servo.h>
#include <math.h>

const int SERVO_PIN = 10;  // 舵机引脚
const int DIR_PIN   = 46;  // 电机方向
const int MOTOR_PIN = 21;  // 电机速度

RPLidar lidar;
Servo servo;

int   servo_angle  = 90;    // 指令角（度）
int   servo_offset = -10;   // 安装偏差
double e_last      = 0;
int   count        = 0;

double distances[360] = {0};
const double min_distance = 0.05;
const double max_distance = 1.0;

// ---- Stanley 参数与保护 ----
const double k_stanley = 0.8;   // 横向误差增益 k
const double v_eps     = 0.1;   // 防止除零
const double steer_gain_deg = 50.0; // 将弧度 δ 放大到舵机度数的比例（可调）
const int    SERVO_MIN = 40;
const int    SERVO_MAX = 140;

// 速度联动（转角越大越慢）
const int    V_BASE  = 150;  // 底速占空比
const int    V_GAIN  = 50;   // 速度提升幅度
const double GAUSS_DEN = 400.0; // 高斯分母：= 2*sigma^2，sigma≈14.14°
const int    PWM_MIN = 0, PWM_MAX = 255;

static int last_velocity_pwm = V_BASE; // 用于估算 v

// 角度归一化到 [-pi, pi]
double wrapAngle(double a){
  return atan2(sin(a), cos(a));
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

    if (startBit) { // 每圈处理一次
      if (count >= 50) {
        // ------- 找最大间隙中点 -------
        double x1=0,y1=0, x2=0,y2=0;
        double max_spacing = 0;
        double xc=0, yc=0;
        bool   have_prev = false;
        bool   found = false;

        // ROI: 45..315（与你原始一致）
        for (int angle_deg = 45; angle_deg < 315; angle_deg++) {
          double d = distances[angle_deg];
          if (d > min_distance && d < max_distance) {
            double rad = angle_deg * M_PI / 180.0;
            x2 = cos(rad) * d;
            y2 = sin(rad) * d;

            if (have_prev) {
              double dx = x2 - x1, dy = y2 - y1;
              double spacing = sqrt(dx*dx + dy*dy);
              if (spacing > max_spacing) {
                max_spacing = spacing;
                xc = 0.5*(x1 + x2);
                yc = 0.5*(y1 + y2);
                found = true;
              }
            }
            x1 = x2; y1 = y2;
            have_prev = true;
          } else {
            have_prev = false; // 断点
          }
        }

        // ------- Stanley 控制 -------
        int velocity_pwm = V_BASE; // 默认
        if (found) {
          // 参考方向（切向）用指向中点的方向近似
          double psi_t = atan2(yc, xc); // 参考方向
          double psi   = M_PI;          // 车辆朝向（按你的坐标系假定）

          // 航向误差
          double theta_e = wrapAngle(psi_t - psi);

          // 横向误差（车辆坐标系下的 y 值，左正右负）
          double e_cte = yc;

          // 速度估计（无编码器时用 PWM 近似到 [0,1]）
          double v_est = 0.1 + 0.9 * ( (double)last_velocity_pwm / 255.0 );

          // Stanley 控制律
          double delta_rad = theta_e + atan( (k_stanley * e_cte) / (v_est + v_eps) );

          // 映射到舵机角度（围绕 90°）
          int servo_cmd = 90 + (int)(steer_gain_deg * delta_rad);
          // 限幅
          if (servo_cmd < SERVO_MIN) servo_cmd = SERVO_MIN;
          if (servo_cmd > SERVO_MAX) servo_cmd = SERVO_MAX;

          // 写舵机（考虑安装偏差）
          servo_angle = servo_cmd;
          servo.write(servo_angle + servo_offset);

          // --- 速度与转角联动（考虑 offset 的相对中位） ---
          int servo_center = 90 + servo_offset;
          int steer_err    = (servo_angle + servo_offset) - servo_center;
          double gain = exp( - ( (double)steer_err * (double)steer_err ) / GAUSS_DEN );
          velocity_pwm = V_BASE + (int)(V_GAIN * gain);
          if (velocity_pwm < PWM_MIN) velocity_pwm = PWM_MIN;
          if (velocity_pwm > PWM_MAX) velocity_pwm = PWM_MAX;

          analogWrite(MOTOR_PIN, velocity_pwm);
          last_velocity_pwm = velocity_pwm;

          // 调试输出
          Serial.printf("xc=%.3f yc=%.3f | psi_t=%.2fdeg theta_e=%.2fdeg e_cte=%.3f v_est=%.2f | delta=%.2fdeg servo=%d vel=%d\r\n",
                        xc, yc,
                        psi_t*180.0/M_PI, theta_e*180.0/M_PI, e_cte, v_est,
                        delta_rad*180.0/M_PI, servo_angle + servo_offset, velocity_pwm);

        } else {
          // 无有效目标：降速、舵机回中
          int servo_center = 90; // 不加 offset，offset 在 write 里加
          servo_angle = servo_center;
          servo.write(servo_angle + servo_offset);

          velocity_pwm = V_BASE; // 或者更低，甚至 0
          analogWrite(MOTOR_PIN, velocity_pwm);
          last_velocity_pwm = velocity_pwm;

          Serial.println("No valid gap found -> center steer, slow down.");
        }

        count = 0;
      }
    }
  } else {
    lidar.startScan();
  }
}
