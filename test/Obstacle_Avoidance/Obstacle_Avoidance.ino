#define BlueSerial Serial
#define EN1 6
#define EN1_RATE 1.0//PWM调整率
#define IN1 9
#define IN2 8
#define EN2 5
#define EN2_RATE 0.95//PWM调整率
#define IN3 3
#define IN4 2
#define PIN_XJ_L  11
#define PIN_XJ_R  10

#define PIN_SERVO 12
#define PIN_TRIG A1
#define PIN_ECHO A0

#define DISC_VALVE 40//超声波测距阈值 60 40
#define DISC_VALVE1 100//摇头直线距离

#define MIN_VALVE 20  //10
#define MIN_VALVE1 30  //20
#define back_time 400


#include <Servo.h>

volatile int speed_forward;
volatile int speed_trun;
volatile float dist;
volatile float distf;
int fflag=0;
int cflag=0;
Servo servo_12;

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

void forward(int time) {
  //小车前进
  //左轮
  motor(1, 1, speed_forward);
  //右轮
  motor(2, 1, speed_forward);
  delay(time);
  //stop();

}

void back(int time) {
  //小车后退
  //左轮
  motor(1, -1, speed_forward);
  //右轮
  motor(2, -1, speed_forward);
  //Serial.println("b");
  delay(time);
}

void left(int time) {
  //小车左后转
  //左轮
  motor(1, -1, speed_trun);
  //右轮
  motor(2, 1, speed_trun);
  //Serial.println("l");
  delay(time);
}

void right(int time) {
  //小车右后转
  //左轮
  motor(1, 1,speed_trun);
  //右轮
  motor(2, -1, speed_trun);
  //Serial.println("r");
  delay(time);
}

void stop() {
  //小车停车
  //左轮
  motor(1, 0, speed_trun);
  //右轮
  motor(2, 0, speed_trun);
  //Serial.println("s");
}
volatile int on;

int find(){
    servo_12.write(175);  //160
    delay(120);            ///////////////////////////////////////////////////150
    int distance_left_1 = checkdistance(PIN_TRIG, PIN_ECHO);
    servo_12.write(90);
    delay(120);             /////////////////////////////////////////////////////150
    int distance_on_1 = checkdistance(PIN_TRIG, PIN_ECHO);
    servo_12.write(5);   //20
    delay(120);             /////////////////////////////////////////////////////150
    int distance_right_1 = checkdistance(PIN_TRIG, PIN_ECHO);
    servo_12.write(90);
    delay(120); 

    if((distance_left_1 <DISC_VALVE1 || distance_right_1<DISC_VALVE1 || distance_on_1<DISC_VALVE1 )&&!(distance_left_1 >100 && distance_on_1 >50 && distance_right_1>50 || distance_left_1 >50 && distance_on_1 >100 && distance_right_1>50 || distance_left_1 >50 && distance_on_1 >50 && distance_right_1>100)){
      stop();
      //back(800);
      if(distance_left_1 > distance_right_1){
        left(300);
      }else{
        right(300);
      }
      stop(); 
      on = distance_on_1;
      return 0;
    }
    return  1;
}

int lastdirection; //0 left 1 right 2 on
void choose(){
  fflag=0;
    stop();
    servo_12.write(135);  //165  175
    delay(200);            ///////////////////////////////////////////////////
    int distance_left = checkdistance(PIN_TRIG, PIN_ECHO);
    servo_12.write(45);   //15  5
    delay(300);             /////////////////////////////////////////////////////
    int distance_right = checkdistance(PIN_TRIG, PIN_ECHO);
    servo_12.write(90);
    delay(200);             /////////////////////////////////////////////////////
    int distance_on = checkdistance(PIN_TRIG, PIN_ECHO);


    
    if(distance_left <MIN_VALVE1 || distance_right<MIN_VALVE1 || distance_on<MIN_VALVE1){
      stop();
      back(back_time);
      //delay(800);
      forward(50);    ////////////////
      //delay(50);    /////////////////////////
      stop();
      if(distance_left > distance_right){
        left(500);
      }else{
        right(500);
      }

      //delay(500);
      fflag=0;
      cflag=0;
      stop();
    
    
    } 
    // else if(distance_left <MIN_VALVE || distance_right<MIN_VALVE ){
    //     back();
    //     delay(500);
        
    //     stop();
    // }
    // else if(distance_on>DISC_VALVE && ){
    //   forward();
    //   fflag=1;
    //   cflag=0;
    //   delay(100);
    //   dist=distance_on;
    // }
    else if( distance_left >100 && distance_on >50 && distance_right>50 || distance_left >50 && distance_on >100 && distance_right>50 || distance_left >50 && distance_on >50 && distance_right>100){
      
      
      //forward();    ////////////////xinjiade
      if(distance_left>900&&distance_right<900){
        left(500);
        //delay(1000);
        //back();
        //delay(500);
        lastdirection=1;
        stop();

      }else if(distance_right>900&&distance_left<900){
        right(400);
        //delay(1000);
        //back();
        //delay(500);
        lastdirection=0;
        stop();
      }else{
      //else if(distance_left >100 && distance_on >100 && distance_right>100){//
        forward(0);
        while(find());
        lastdirection=2;
      // }else{
      //   forward(0);
      //   while(find());
      //   lastdirection=2;        
        fflag=1;
        dist=on;//////////////
      
      }
      
      //stop();
    }
    
    else if((distance_left >DISC_VALVE || distance_right>DISC_VALVE)&& distance_on>30 ){
      if (distance_left > distance_right) {
        // back();
        // delay(600);
        //left();
        if(lastdirection==1){
          right(400);
          //delay(400);
        }
        else if(distance_left - distance_right>50){
          left(750);
          lastdirection=0;
        }else{
          left(500);
          lastdirection=0;
        }
        stop();
      
      } else {
        // back();
        // delay(600);
        //right();
        
        if(lastdirection==0){
          left(400);
          //delay(400);
        }
        else if(distance_right - distance_left>50){
          right(750);
          lastdirection=1;
        }else{
          right(500);
          lastdirection=1;
        }
        stop();
      }

    }else{
      if(distance_left > distance_right){  ////////////////xinjiade
        left(400);
      }else{
        right(400);
      }//delay(50);
      fflag=1;
      dist=distance_on;
      stop();
      lastdirection=2;
    }
    // servo_12.write(90);
    // delay(200);

}

float checkdistance(int pinTrig, int pinEcho) {
  digitalWrite(pinTrig, LOW);
  delayMicroseconds(2);
  digitalWrite(pinTrig, HIGH);
  delayMicroseconds(10);
  digitalWrite(pinTrig, LOW);
  unsigned long duration = pulseIn(pinEcho, HIGH, 100000);

  if (duration == 0) {
    Serial.println("Error: No echo received!");
    return -1.0; // 返回 -1 表示测量失败
  }
  delay(10);
  float distance = duration / 58.0f; // 标准计算公式（cm）

  // Serial.print("Duration (μs): ");
  // Serial.print(duration);
  // Serial.print(" → Distance (cm): ");
  Serial.println(distance);

  return distance;
}

int count;

void csbModule() {
    
  //超声波避障模块
  servo_12.write(90);
  //delay(200);
  dist = checkdistance(PIN_TRIG, PIN_ECHO);
  // Serial.println(dist);
  if(dist > 0 && dist <MIN_VALVE){
    stop();
    back(back_time);
    //delay(800);
    forward(50);   //////////////
    //delay(50);////////////////////
    stop();
    left(500);
    //delay(500);
    fflag=0;
    cflag=0;
    stop();
  }
  // else if(dist==0){
  //   forward(0);
  //   fflag=1;
  //   cflag=0;
  //   //delay(50);    /////////////////////////////////!!!!!
  // }
  else if (dist > DISC_VALVE) {
    // if(dist > 500){
    //   back();
    //   delay(750);
    //   left();
    //   delay(500);
    //   stop();
    //   fflag=0;

    if((dist - distf > 50 && fflag==1 && cflag==0) || (dist >150 && distf > 150 && cflag==0)){
      Serial.print("CHoose");
      Serial.println(++count);
      //back();
      //delay(500);
      choose();
      cflag=1;
    }else if(dist>60){    //else{  else if会有问题
      forward(0);
      //delay(20);      ///////////////////////////
      distf=dist;
      fflag=1;
      cflag=0;
    }
  } else if(dist!=-1.0 && cflag==0){
    cflag=1;
    choose();
    
  }else{
    back(back_time);
    //delay(500);
    stop();  ////////////////
    forward(100); //////////////////
    //delay(100); ////////////////////
    stop();      //////////////////
    left(300); 
    //delay(300); ////////////////////degai
    stop();        ///////////////////
  }

}

void setup(){
  speed_forward = 140;
  speed_trun=160;
  dist = 0;
  distf=0;
  lastdirection=2;
  Serial.begin(9600);
  servo_12.attach(12);
  pinMode(EN1, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(EN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(PIN_SERVO, OUTPUT);
  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
  //舵机初始位置
  servo_12.write(90);
  delay(3000);
}

void loop(){
  csbModule();
}