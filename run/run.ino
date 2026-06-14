/*
 * 项目名称: 面向复杂对抗环境的多模态感知自主移动机器人
 * 核心功能: 动态避障 (优先级1) > 趋光追踪 (优先级2) > 高精度循迹 (优先级3)
 */

#include <Servo.h>
#include <PID_v1.h>
#include <Wire.h>
#include <ADS1115_WE.h> 

// ================= 引脚统一定义 =================
// 电机驱动引脚 [cite: 1, 92]
#define EN1 6
#define IN1 9
#define IN2 8
#define EN2 5
#define IN3 3
#define IN4 2
#define EN1_RATE 1.0  // PWM 动态校准系数 (修正电机公差) [cite: 1]
#define EN2_RATE 0.95 // PWM 动态校准系数 [cite: 1]

// 超声波与舵机引脚 [cite: 1]
#define PIN_SERVO 12
#define PIN_TRIG A1 
#define PIN_ECHO A0

// 光敏传感器引脚 (解决原代码A1引脚冲突，移至A2, A3)
#define PIN_LIGHT_L A2
#define PIN_LIGHT_R A3

// ADS1115 I2C 地址 [cite: 94]
#define I2C_ADDRESS 0x48

// ================= 全局阈值与对象 =================
#define DISC_VALVE 40    // 避障触发阈值(cm) [cite: 1]
#define LIGHT_THRESHOLD 200 // 进入巡光模式的光照差值或绝对值阈值

Servo servo_PTZ; // 舵机云台 [cite: 2]
ADS1115_WE adc = ADS1115_WE(I2C_ADDRESS); [cite: 94]

// PID 参数与对象 [cite: 69, 71]
double Setpoint, Input, Output;
double Kp_left = 0.6, Ki_left = 0.0075, Kd_left = 0.8;
double Kp_right = 0.5, Ki_right = 0.01, Kd_right = 0.7;
PID pidLeft(&Input, &Output, &Setpoint, Kp_left, Ki_left, Kd_left, DIRECT); [cite: 71]
PID pidRight(&Input, &Output, &Setpoint, Kp_right, Ki_right, Kd_right, DIRECT); [cite: 72]

// 状态机枚举
enum RobotState {
  STATE_LINE_TRACK,
  STATE_LIGHT_TRACK,
  STATE_OBSTACLE_AVOID
};
RobotState currentState = STATE_LINE_TRACK;

// 循迹误差记忆 [cite: 98]
int errorLast = 0;
int irSensors = B000; [cite: 96]

void setup() {
  Serial.begin(9600);
  Wire.begin();
  Wire.setClock(400000); // I2C 高频采样 [cite: 144]

  // 初始化 ADS1115 [cite: 144]
  if(!adc.init()){ Serial.println("ADS1115 Error!"); }
  adc.setVoltageRange_mV(ADS1115_RANGE_6144); [cite: 144]
  adc.setConvRate(ADS1115_860_SPS); [cite: 144]
  adc.setMeasureMode(ADS1115_SINGLE); [cite: 144]

  // 初始化电机与传感器
  pinMode(EN1, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT); [cite: 65]
  pinMode(EN2, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT); [cite: 65]
  pinMode(PIN_TRIG, OUTPUT); pinMode(PIN_ECHO, INPUT); [cite: 65]
  
  servo_PTZ.attach(PIN_SERVO); [cite: 65]
  servo_PTZ.write(90); [cite: 66]

  // PID 初始化 [cite: 75, 76]
  Setpoint = 0;
  pidLeft.SetMode(AUTOMATIC); pidLeft.SetOutputLimits(-60, 60);
  pidRight.SetMode(AUTOMATIC); pidRight.SetOutputLimits(-60, 60);
  
  delay(1000);
}

void loop() {
  // 1. 感知层：获取多模态传感器数据
  float dist = checkDistance(PIN_TRIG, PIN_ECHO);
  int lightLeft = 1023 - analogRead(PIN_LIGHT_L);
  int lightRight = 1023 - analogRead(PIN_LIGHT_R);

  // 2. 决策层：启发式状态机切换
  if (dist > 0 && dist < DISC_VALVE) {
    currentState = STATE_OBSTACLE_AVOID; // 最高优先级：景深矩阵避障
  } else if (abs(lightLeft - lightRight) > LIGHT_THRESHOLD || (lightLeft > 800 && lightRight > 800)) {
    currentState = STATE_LIGHT_TRACK;    // 次优先级：捕获特异光源
  } else {
    currentState = STATE_LINE_TRACK;     // 默认行为：全局高频循迹
  }

  // 3. 执行层：根据状态触发对应行为
  switch (currentState) {
    case STATE_OBSTACLE_AVOID:
      executeObstacleAvoidance();
      break;
    case STATE_LIGHT_TRACK:
      executeLightTracking(lightLeft, lightRight);
      break;
    case STATE_LINE_TRACK:
      executeLineTracking();
      break;
  }
}

// ================= 底层运动控制 (带死区限制与动态比例) =================
void motorControl(int speedL, int speedR) {
  // 限制速度与公差校准
  speedL = constrain(speedL, -255, 255) * EN1_RATE;
  speedR = constrain(speedR, -255, 255) * EN2_RATE;

  // 左电机驱动 [cite: 8, 9, 10]
  if(speedL > 0) { digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW); }
  else if(speedL < 0) { digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH); }
  else { digitalWrite(IN1, LOW); digitalWrite(IN2, LOW); }
  analogWrite(EN1, abs(speedL)); [cite: 9]

  // 右电机驱动
  if(speedR > 0) { digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW); }
  else if(speedR < 0) { digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH); }
  else { digitalWrite(IN3, LOW); digitalWrite(IN4, LOW); }
  analogWrite(EN2, abs(speedR));
}

// ================= 模块1：云台矩阵扫描避障 =================
void executeObstacleAvoidance() {
  motorControl(0, 0); // 紧急制动 [cite: 23]
  
  // 云台扫描构建景深矩阵 [cite: 23, 24, 25]
  servo_PTZ.write(135); delay(200);
  float distLeft = checkDistance(PIN_TRIG, PIN_ECHO); [cite: 23]
  servo_PTZ.write(45); delay(300);
  float distRight = checkDistance(PIN_TRIG, PIN_ECHO); [cite: 24]
  servo_PTZ.write(90); delay(200); [cite: 24]

  // 启发式决策脱困 [cite: 26, 27]
  motorControl(-100, -100); delay(400); // 倒车 [cite: 26]
  if (distLeft > distRight) {
    motorControl(-150, 150); delay(300); // 左自旋 [cite: 20]
  } else {
    motorControl(150, -150); delay(300); // 右自旋 [cite: 21]
  }
}

// ================= 模块2：双PID趋光追踪 =================
void executeLightTracking(int lightL, int lightR) {
  Input = (lightL + 40) - lightR; // 偏差计算与补偿 [cite: 79]
  
  if (Input < 0) pidLeft.Compute(); [cite: 80]
  else pidRight.Compute(); [cite: 81]

  int baseSpeed = 115; [cite: 72]
  int speedL = baseSpeed, speedR = baseSpeed;

  if (Output < 0) {
    speedL -= abs(Output); speedR += abs(Output); [cite: 84]
  } else {
    speedL += abs(Output); speedR -= abs(Output); [cite: 86]
  }
  motorControl(speedL, speedR);
}

// ================= 模块3：高频循迹与状态机回正 =================
void executeLineTracking() {
  // ADC 提取黑线特征 [cite: 123, 124]
  int s0 = readADCChannel(ADS1115_COMP_0_GND); [cite: 123]
  int s1 = readADCChannel(ADS1115_COMP_1_GND); [cite: 124]
  int s2 = readADCChannel(ADS1115_COMP_2_GND); [cite: 124]
  irSensors = (s0 << 2) | (s1 << 1) | s2; [cite: 126]

  // 二进制状态机决策 [cite: 128]
  switch (irSensors) {
    case B000: // 脱线，根据误差记忆回正 [cite: 128]
      if (errorLast < 0) motorControl(-100, 100); // 原地左转 [cite: 128]
      else motorControl(100, -100);             // 原地右转 [cite: 129]
      break;
    case B100: case B110: 
      motorControl(-30, 120); errorLast = -1; break; [cite: 131, 132]
    case B010: 
      motorControl(120, 120); errorLast = 0; break; [cite: 136]
    case B001: case B011: 
      motorControl(120, -30); errorLast = 1; break; [cite: 137, 138]
    case B111:
      motorControl(100, 100); break; [cite: 140]
  }
}

// ================= 辅助函数 =================
float checkDistance(int pinTrig, int pinEcho) {
  digitalWrite(pinTrig, LOW); delayMicroseconds(2);
  digitalWrite(pinTrig, HIGH); delayMicroseconds(10);
  digitalWrite(pinTrig, LOW); [cite: 49]
  unsigned long duration = pulseIn(pinEcho, HIGH, 30000);
  if (duration == 0) return 999.0;
  return duration / 58.0f; // 多普勒解算距离 [cite: 51]
}

int readADCChannel(ADS1115_MUX channel) {
  adc.setCompareChannels(channel);
  adc.startSingleMeasurement(); [cite: 145]
  while (adc.isBusy()) { delay(1); } [cite: 146]
  return adc.getResult_V() > 1.0 ? 1 : 0; // 电压阈值二值化提取黑线 [cite: 147]
}