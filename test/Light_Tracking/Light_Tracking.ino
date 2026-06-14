#include <PID_v1.h>

// 电机引脚定义
int enablePin1 = 9;    // 左侧电机PWM
int enablePin2 = 10;   // 右侧电机PWM
int in1Pin = 5;        // 左侧电机方向1
int in2Pin = 6;        // 左侧电机方向2
int in3Pin = 7;        // 右侧电机方向1
int in4Pin = 8;        // 右侧电机方向2

// 传感器定义
int LeftSensor = A1;
int RightSensor = A2;

// 左转PID参数（增强响应）
double Kp_left = 0.6;   // 提高比例增益
double Ki_left = 0.0075;  // 较小积分
double Kd_left = 0.8;   // 适当提高微分

// 右转PID参数（保持原样）
double Kp_right = 0.5;
double Ki_right = 0.01;
double Kd_right = 0.7;

// PID变量
double Setpoint, Input, Output;

// 双PID控制器
PID pidLeft(&Input, &Output, &Setpoint, Kp_left, Ki_left, Kd_left, DIRECT);
PID pidRight(&Input, &Output, &Setpoint, Kp_right, Ki_right, Kd_right, DIRECT);

// 优化后的电机控制参数
const int baseSpeed = 115;     // 提高基础速度(原80)
const int maxSpeed = 190;      // 提高最大速度(原120)
const int minSpeed = 70;       // 提高最小速度(原40)
const int sensorThreshold = 30; // 降低阈值使响应更灵敏

// 滤波参数
float leftFiltered = 0;
float rightFiltered = 0;
const float filterFactor = 0.3; // 增强滤波效果

void setup() {
  Serial.begin(9600);
  
  // 初始化电机控制引脚
  pinMode(in1Pin, OUTPUT);
  pinMode(in2Pin, OUTPUT);
  pinMode(in3Pin, OUTPUT); 
  pinMode(in4Pin, OUTPUT);
  pinMode(enablePin1, OUTPUT);
  pinMode(enablePin2, OUTPUT);
  
  // PID初始化
  Setpoint = 0;
  pidLeft.SetMode(AUTOMATIC);
  pidLeft.SetOutputLimits(-60, 60); // 缩小输出范围减少震荡
  pidLeft.SetSampleTime(20); // 增加采样时间到15ms
  pidRight.SetMode(AUTOMATIC);
  pidRight.SetOutputLimits(-60, 60); // 缩小输出范围减少震荡
  pidRight.SetSampleTime(20); // 增加采样时间到15ms
  delay(1000);
}

void loop() {
  // 1. 读取并增强滤波  L比R小30
  int leftRaw = 1023 - analogRead(LeftSensor);
  int rightRaw = 1023 - analogRead(RightSensor);
  
  leftFiltered = leftFiltered * (1-filterFactor) + leftRaw * filterFactor;
  rightFiltered = rightFiltered * (1-filterFactor) + rightRaw * filterFactor;
  
  // 3. 计算偏差（左转时增加补偿）
  Input = (leftFiltered + 40) - rightFiltered;  // 补偿左传感器
  
  
  // 4. 根据转向选择PID
  if (Input < 0) {  // 左转
    pidLeft.Compute();
  } else {          // 右转
    pidRight.Compute();
  }
  
  // 4. 动态速度调整
  int leftSpeed = baseSpeed;
  int rightSpeed = baseSpeed;
  
  if(abs(Input) > sensorThreshold) {
    // 使用更平滑的速度映射
    if(Output < 0) { // 左转
      int adjustment = map(abs(Output), 0, 80, 0, maxSpeed-baseSpeed+30);
      adjustment = constrain(adjustment, 0, maxSpeed-baseSpeed);
      leftSpeed = baseSpeed - adjustment;
      rightSpeed = baseSpeed + adjustment;
    } 
    else { // 右转
      int adjustment = map(abs(Output), 0, 80, 0, maxSpeed-baseSpeed);
      adjustment = constrain(adjustment, 0, maxSpeed-baseSpeed);
      leftSpeed = baseSpeed + adjustment;
      rightSpeed = baseSpeed - adjustment;
    }
  }
  


  
  // 确保速度在限制范围内
  leftSpeed = constrain(leftSpeed, minSpeed, maxSpeed);
  rightSpeed = constrain(rightSpeed, minSpeed, maxSpeed);
  
  // 5. 设置电机
  setMotor(leftSpeed, rightSpeed, false);
  
  // 6. 调试输出
  // Serial.print("L:");
  // Serial.print(leftFiltered);
  // Serial.print(" R:");
  // Serial.print(rightFiltered); 
  // Serial.print(" Out:");
  // Serial.print(Output);
  // Serial.print(" LSpd:");
  // Serial.print(leftSpeed);
  // Serial.print(" RSpd:");
  // Serial.println(rightSpeed);
  
  delay(15); // 与PID采样时间一致
}

void setMotor(int leftSpeed, int rightSpeed, boolean reverse) {
  analogWrite(enablePin1, leftSpeed);
  digitalWrite(in1Pin, !reverse);
  digitalWrite(in2Pin, reverse);
  analogWrite(enablePin2, rightSpeed);
  digitalWrite(in3Pin, !reverse);
  digitalWrite(in4Pin, reverse);
}