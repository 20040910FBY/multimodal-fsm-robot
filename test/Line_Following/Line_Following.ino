#define EN1_RATE 1.0//PWM调整率
#define EN2_RATE 0.95//PWM调整率
/*
Line following control of two Bi - directional Motors
*/
#include<ADS1115_WE.h> 
#include<Wire.h>
#define I2C_ADDRESS 0x48

int EN1 = 6;
int EN2 = 5;
int IN1 = 9;
int IN2 = 8;
int IN3 = 3;
int IN4 = 2;

ADS1115_WE adc = ADS1115_WE(I2C_ADDRESS);
/* Define the pins for the IR sensors */
//const int irPins[3] = {12, 11, 10};
/* Define values for the IR Sensor readings */
int irSensorDigital[3] = {0, 0, 0};
int threshold = 200; // IR sensor threshold value for line detection
// binary representation of the sensor reading
// 1 when the sensor detects the line, 0 otherwise
int irSensors = B000;
// A score to determine deviation from the line [-180 ; +180].
// Negative means the robot is left of the line.
int error = 0;
int errorLast = 0; //  store the last value of error

volatile int speed;


void motor(int m, int dir, int speed) {
  //马达控制模块：
  //m: 马达编号，1-左前轮、2-右前轮
  //dir: 转动方向，1 前进、-1 后退 、0 停车
  //speed: PWM速度
  //
  int pinEN = 0;
  int pinIN1 = 0;
  int pinIN2 = 0;
  float enRate = 0;
  switch (m) {
   case 1:
    pinEN = EN1;
    pinIN1 = IN1;
    pinIN2 = IN2;
    enRate = EN1_RATE;
    break;
   case 2:
    pinEN = EN2;
    pinIN1 = IN3;
    pinIN2 = IN4;
    enRate = EN2_RATE;
    break;
   default:
    return;
    break;
  }
  switch (dir) {
   case 0:
    //停车
    digitalWrite(pinIN1, 0);
    digitalWrite(pinIN2, 0);
    analogWrite(pinEN, 0);
    break;
   case 1:
    //前进
    digitalWrite(pinIN1, 1);
    digitalWrite(pinIN2, 0);
    analogWrite(pinEN, speed * enRate);
    break;
   case -1:
    //后退
    digitalWrite(pinIN1, 0);
    digitalWrite(pinIN2, 1);
    analogWrite(pinEN, speed * enRate);
    break;
   default:
    return;
    break;
  }
}

void forward() {
  //小车前进
  //左轮
  motor(1, 1, speed+70);
  //右轮
  motor(2, 1, speed+70);
}

void lowforward() {
  //小车前进
  //左轮
  motor(1, 1, speed);
  //右轮
  motor(2, 1, speed);
}

void back() {
  //小车后退
  //左轮
  motor(1, -1, speed);
  //右轮
  motor(2, -1, speed);
}

void left() {
  //小车左转
  //左轮
  motor(1, -1, speed+80);
  //右轮
  motor(2, 1, speed+80);
}


void sleft() {
  //小车轻微左转
  //左轮
  motor(1, -1, speed+30);
  //右轮
  motor(2, 1, speed+30);
}

void ssleft() {
  //小车及其轻微左转
  //左轮
  motor(1, -1, speed+30);
  //右轮
  motor(2, 1, speed+30);
}

void right() {
  //小车右转
  //左轮
  motor(1, 1, speed+80);
  //右轮
  motor(2, -1, speed+80);
}

void sright() {
  //小车轻微右转
  //左轮
  motor(1, 1, speed+30);
  //右轮
  motor(2, -1, speed+30);
}

void ssright() {
  //小车及其轻微右转
  //左轮
  motor(1, 1, speed+30);
  //右轮
  motor(2, -1, speed+30);
}

// void sharpleft(){
//   //小车左转
//   //左轮
//   motor(1, -1, speed+90);
//   //右轮
//   motor(2, 1, speed+90);
//   delay(300);
// }

// void sharpright(){
//   //小车左转
//   //左轮
//   motor(1, 1, speed+100);
//   //右轮
//   motor(2, -1, speed+100);
//   delay(300);
// }

void stop() {
  //小车停车
  //左轮
  motor(1, 0, speed);
  //右轮
  motor(2, 0, speed);
}

void Scan() {
  // Initialize the sensors
  irSensors = B000;
  for (int i = 0; i < 3; i++) {
    if(i==0){
      irSensorDigital[i] = readChannel(ADS1115_COMP_0_GND);//digitalRead(irPins[i]);
    }else if(i==1){
      irSensorDigital[i] = readChannel(ADS1115_COMP_1_GND);//digitalRead(irPins[i]);
    }else if(i==2){
      irSensorDigital[i] = readChannel(ADS1115_COMP_2_GND);//digitalRead(irPins[i]);
    }

    
    int b = 2 - i;
    irSensors = irSensors + (irSensorDigital[i] << b);
  }
}

int stopflag=0;
int count=0;
int lowcount=0;
void UpdateDirection(int xjspeed) {
    speed=xjspeed;
    errorLast = error;
    Serial.println(irSensors);
    switch (irSensors) {
        case B000:  // no sensor detects the line
            if (errorLast < 0) {
                left();               
                error = -1;
            } else if (errorLast > 0) {
                right();
                error = 1;
            }
            count=0;
            lowcount=0;
            stopflag=1;
            break;
        case B100:  // left sensor on the line
            sleft(); 
            error = -1;
            count=0; 
            lowcount=0;  
            stopflag=1;        
            break;
        case B110:
            ssleft();
            error = -1;
            count=0;
            lowcount=0;
            stopflag=1;
            break;
        case B010:
            if(count<30){
              forward();
              count++;
            }else if(lowcount<4){
              lowforward();
              lowcount++;
            }else if(lowcount>4){
              forward();
              count=0;
            }
            //forward();

            error = 0;
            stopflag=1;
            break;
        case B011:
            ssright();
            error = 1;
            count=0;
            lowcount=0;
            stopflag=1;
            break;
        case B001:  // right sensor on the line
            sright();
            error = 1;
            count=0;
            lowcount=0;
            stopflag=1;
            break;
        case B111:
            error = 1;
            stopflag=1;
            forward();
            break;
        default:
            if(stopflag==1){
              stop();
              stopflag=0;
            }else{
            stopflag=1;
            }
            error = errorLast;
            count=0;
            lowcount=0;
    }

}

void setup() {
    Wire.begin();
    Wire.setClock(400000);
    Serial.begin(9600);
    if(!adc.init()){
    Serial.println("ADS1115 not connected!");
    }
    adc.setVoltageRange_mV(ADS1115_RANGE_6144);
    adc.setConvRate(ADS1115_860_SPS);
    adc.setMeasureMode(ADS1115_SINGLE);
    
    speed = 90;
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
    pinMode(EN1, OUTPUT);
    pinMode(EN2, OUTPUT);
    Serial.begin(9600);
}

void loop() {
    Scan();
    UpdateDirection(speed);
}

int readChannel(ADS1115_MUX channel) {
  adc.setCompareChannels(channel);
  adc.startSingleMeasurement();  // SINGLE 模式采样

  while (adc.isBusy()) {
    delay(1);  // 等待采样完成
  }

  float voltage = adc.getResult_V();  // 获取电压（也可以用 getResult_mV）
  return voltage > 1;  // 超过 1V 代表检测到黑线
}