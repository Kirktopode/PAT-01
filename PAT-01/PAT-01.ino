#include "Wire.h"
#include "I2Cdev.h"
#include "MPU9250.h"
#include <math.h>
#include <JVector.h>

const int LTRN = 5;
const int RTRN = 16;
const int FWRD = 15;
const int BWRD = 14;
const int BUTTON = 7;
//const int LED = 17;

const float MPS = 1.651;

int32_t mx_min = 32766, mx_max = -32766, mx_bias;
int32_t my_min = 32766, my_max = -32766, my_bias;
int32_t mz_min = 32766, mz_max = -32766, mz_bias;

int32_t cal_min, cal_max;

MPU9250 mag;
I2Cdev I2C_M;

enum ThrottleState{
  FORWARD, REVERSE, STILL
};

void setup() {
  // put your setup code here, to run once:

  pinMode(LTRN, OUTPUT);
  pinMode(RTRN, OUTPUT);
  pinMode(FWRD, OUTPUT);
  pinMode(BWRD, OUTPUT);
//  pinMode(LED, OUTPUT);
  pinMode(BUTTON, INPUT);

  Wire.begin();
  mag.initialize();

  Serial.begin(9600);
}

float heading, desiredHeading;
int wpIndex = 1;
float headingChange;
const int WP_LENGTH = 5;
const float DEADBAND = 15; //degree range for acceptable forward heading.
unsigned long startTime;
ThrottleState throttle = STILL;
JVector pos(0,0);

JVector waypoints[] = {JVector(0,0), JVector(10, 0), JVector(10,10), JVector(0,10), JVector(0,0)};

void steerLeft(){
  digitalWrite(RTRN, LOW);
  digitalWrite(LTRN, HIGH);
}

void steerRight(){
  digitalWrite(RTRN, HIGH);
  digitalWrite(LTRN, LOW);
}

void steerStraight(){
  digitalWrite(RTRN, LOW);
  digitalWrite(LTRN, LOW);
}

void motorForward(){
  digitalWrite(FWRD, HIGH);
  digitalWrite(BWRD, LOW);
  throttle = FORWARD;
}

void motorBack(){
  digitalWrite(FWRD, LOW);
  digitalWrite(BWRD, HIGH);
  throttle = REVERSE;
}

void motorStop(){
  digitalWrite(FWRD, LOW);
  digitalWrite(BWRD, LOW);
  throttle = STILL;
}

void calibrateMag(){
  int16_t ax, ay,az, gx, gy, gz, mx, my, mz;
//  digitalWrite(LED, LOW);
  while(digitalRead(BUTTON) == LOW){
    steerLeft();
    mag.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
    if(mx > mx_max) mx_max = mx;
    if(mx < mx_min) mx_min = mx;
    if(my > my_max) my_max = my;
    if(my < my_min) my_min = my;
    if(mz > mz_max) mz_max = mz;
    if(mz < mz_min) mz_min = mz;
    Serial.print("Left, ");
    Serial.print(mx);
    Serial.print(", ");
    Serial.print(my);
    Serial.print(", ");
    Serial.print(mz);
    Serial.print(", ");
    Serial.print(mx_max);
    Serial.print(", ");
    Serial.print(my_max);
    Serial.print(", ");
    Serial.print(mz_max);
    Serial.print(", ");
    Serial.print(mx_min);
    Serial.print(", ");
    Serial.print(my_min);
    Serial.print(", ");
    Serial.println(mz_min);
  }
  delay(500);
  while(digitalRead(BUTTON) == LOW){
    steerRight();
    mag.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
    if(mx > mx_max) mx_max = mx;
    if(mx < mx_min) mx_min = mx;
    if(my > my_max) my_max = my;
    if(my < my_min) my_min = my;
    if(mz > mz_max) mz_max = mz;
    if(mz < mz_min) mz_min = mz;
    Serial.print("Right, ");
    Serial.print(mx);
    Serial.print(", ");
    Serial.print(my);
    Serial.print(", ");
    Serial.print(mz);
    Serial.print(", ");
    Serial.print(mx_max);
    Serial.print(", ");
    Serial.print(my_max);
    Serial.print(", ");
    Serial.print(mz_max);
    Serial.print(", ");
    Serial.print(mx_min);
    Serial.print(", ");
    Serial.print(my_min);
    Serial.print(", ");
    Serial.println(mz_min);
  }
  delay(500);
  while(digitalRead(BUTTON) == LOW){
    steerStraight();
    mag.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
    if(mx > mx_max) mx_max = mx;
    if(mx < mx_min) mx_min = mx;
    if(my > my_max) my_max = my;
    if(my < my_min) my_min = my;
    if(mz > mz_max) mz_max = mz;
    if(mz < mz_min) mz_min = mz;
    Serial.print("Straight, ");
    Serial.print(mx);
    Serial.print(", ");
    Serial.print(my);
    Serial.print(", ");
    Serial.print(mz);
    Serial.print(", ");
    Serial.print(mx_max);
    Serial.print(", ");
    Serial.print(my_max);
    Serial.print(", ");
    Serial.print(mz_max);
    Serial.print(", ");
    Serial.print(mx_min);
    Serial.print(", ");
    Serial.print(my_min);
    Serial.print(", ");
    Serial.println(mz_min);
  }
  delay(50);
  startTime = millis();
//  digitalWrite(LED, HIGH);
  
  mx_bias = (mx_min + mx_max) / 2;
  my_bias = (my_min + my_max) / 2;
  mz_bias = (mz_min + mz_max) / 2;

  mx_min -= mx_bias;
  mx_max -= mx_bias;
  my_min -= my_bias;
  my_max -= my_bias;
  mz_min -= mz_bias;
  mz_max -= mz_bias;

  cal_min = (mx_min + my_min + mz_min) / 2;
  cal_max = (mx_max + my_max + mz_max) / 2;
}

void navigate(){
  int16_t ax, ay,az, gx, gy, gz, mx, my, mz;
  mag.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
  int16_t mx_adjusted = map(mx - mx_bias, mx_min, mx_max, cal_min, cal_max);
  int16_t my_adjusted = map(my - my_bias, my_min, my_max, cal_min, cal_max);
  heading = atan2(-my_adjusted,mx_adjusted) * 180 / PI - 180;
  while(heading < 0) heading += 360;
  if(throttle == FORWARD){
    pos += JVector(MPS*sin(heading),MPS*cos(heading));
  }else if(throttle == REVERSE){
    pos -= JVector(MPS*sin(heading),MPS*cos(heading));
  }
  
  Serial.print(mx);
  Serial.print(", ");
  Serial.print(my);
  Serial.print(", ");
  Serial.print(mz);
  Serial.print(", ");
  Serial.print(mx_max);
  Serial.print(", ");
  Serial.print(my_max);
  Serial.print(", ");
  Serial.print(mz_max);
  Serial.print(", ");
  Serial.print(mx_min);
  Serial.print(", ");
  Serial.print(my_min);
  Serial.print(", ");
  Serial.print(mz_min);
  Serial.print(", ");
  Serial.print(mx_bias);
  Serial.print(", ");
  Serial.print(my_bias);
  Serial.print(", ");
  Serial.print(mz_bias);
  Serial.print(", ");
  //Serial.print(desiredHeadings[hIndex]);
  //Serial.print(", ");
  Serial.print(heading);
}

void guide(){

  
  
  if(headingChange > 180) headingChange -= 360;
  if(headingChange < -180) headingChange += 360;
  Serial.print(", ");
  Serial.println(headingChange);
}

//340 - 0 is 340 (left turn)
//160 - 180 is -20 (left turn)

void control(){
  if(millis() - startTime >= 3000){
    digitalWrite(FWRD, LOW);
    if(digitalRead(BUTTON) == HIGH){
      while(digitalRead(BUTTON)==HIGH){
        delay(10);
        startTime = millis();
      }
    }
  }else{
    if(wpIndex >= WP_LENGTH){
      digitalWrite(FWRD, LOW);
      digitalWrite(RTRN, LOW);
      digitalWrite(LTRN, LOW);
      while(true){
      }
    }else if(abs(headingChange) >= DEADBAND / 2){
      //Time to TURN
      digitalWrite(FWRD, HIGH);
      if(headingChange < 0){
        digitalWrite(LTRN, HIGH);
        digitalWrite(RTRN, LOW);
      }else{
        digitalWrite(LTRN, LOW);
        digitalWrite(RTRN, HIGH);
      }
    }else{
      digitalWrite(LTRN, LOW);
      digitalWrite(RTRN, LOW);
  //    digitalWrite(LED, LOW);
  //    delay(500); //Wait a discernible amount of time
  //    digitalWrite(LED, HIGH);
  //    hIndex++;
    }
  }
}

void loop() {
  calibrateMag();
  delay(1000); //give me a second to move my hand
  while(true){
    navigate();
    guide();
    control();
  }
}
