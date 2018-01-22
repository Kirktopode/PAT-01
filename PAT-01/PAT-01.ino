#include "Wire.h" //Used to communicate to the IMU (Accelerometer, gyroscope and magnetometer)
#include "I2Cdev.h" //Helps with the above
#include "MPU9250.h" //Contains a library of functions that streamline communication with the IMU

#include <math.h> //For math (trig functions)
#include <JVector.h> //home-made library for basic 3D vectors. We only use 2 dimensions of it.

const int LTRN = 5; //Left turn pin
const int RTRN = 16; //Right turn pin --if both turn pins are HIGH, no turn is made.
const int FWRD = 15; //Forward throttle pin
const int BWRD = 14; //Reverse throttle pin --if both pins are HIGH, no turn is made.
const int BUTTON = 6; //Button pin --for various steps forward in the robot's state
const int ODOMETER_DIGITAL = 8; //Pin to be used to attach the interrupt for the odometer.
const int ODOMETER_ANALOG = A2;
//const int LED = 17; //Commented out because use of this pin interferes with serial communication.
//Not removed because we may want indicator lights at another stage in development.

const float MPS = 1.651; //A decent estimate for the robot's average speed over 1.5 seconds of operation.
//With an odometer, we can compute movement much more effectively, but that will have to wait.



MPU9250 mag; //Object used to communicate with the IMU
I2Cdev I2C_M; //Object used to initiate I2C communication protocol

enum ThrottleState{
  FORWARD, REVERSE, STILL //symbolic constants used to store whether the robot is moving forward, back, or not at all.
};

void setup() {

  //Setting up timer interrupt
  cli();
  OCR0A = 0xAF;
  TIMSK0 |= _BV(OCIE0A);
  sei();

  pinMode(LTRN, OUTPUT);
  pinMode(RTRN, OUTPUT);
  pinMode(FWRD, OUTPUT);
  pinMode(BWRD, OUTPUT);
//  pinMode(LED, OUTPUT);
  pinMode(BUTTON, INPUT);
  pinMode(ODOMETER_ANALOG,INPUT);
  //Commenting this out because it doesn't work, but does illustrate interrupts pretty well.
  //attachInterrupt(digitalPinToInterrupt(ODOMETER_INTERRUPT_PIN), odometer, RISING);
  Wire.begin();
  mag.initialize();

  Serial.begin(9600); //We initiate Serial communication for logging purposes.
}


const int WP_LENGTH = 5; //Length of the list of waypoints.
const float DEADBAND = 15; //degree range for acceptable forward heading.
//We use a deadband because the steering is only capable of three settings, unlike hobby-grade cars.
const uint16_t UPPER_THRESHOLD = 700; //thresholds used for the odometer
const uint16_t LOWER_THRESHOLD = 300;

const float WHEEL_CIRC_CM = 4.49 * PI;

unsigned long odometerTime = 0;//used for odometer timing
uint16_t odometerMidpoint = 0; //used to identify the midpoint on the odometer
int32_t mx_min = 32766, mx_max = -32766, mx_bias; //these values are used for magnetometer calibration.
int32_t my_min = 32766, my_max = -32766, my_bias;
int32_t mz_min = 32766, mz_max = -32766, mz_bias;

int32_t cal_min, cal_max;

float heading, desiredHeading, headingChange;
int wpIndex = 1; //current index in the waypoints array
unsigned long epochTime; //stored in milliseconds and changed with button presses.
unsigned long halfTurns = 0; //updated via interrupt
unsigned long lastTurns = 0; //holds the previous value for navigate updates
uint16_t threshold = UPPER_THRESHOLD;
ThrottleState throttle = STILL; //current throttle state, beginning at STILL
JVector pos(0,0); //current location, beginning at the origin.

//array of waypoints that the robot will travel to. This is not fully implemented.
JVector waypoints[] = {JVector(0,0), JVector(10, 0), JVector(10,10), JVector(0,10), JVector(0,0)};

/*
 * This interrupt fires every millisecond, polling the current state of the odometer.
 * If the odometer's analog signal is above the threshold, the following happens:
 * 1. halfWheelCount is incremented
 * 2. The threshold is decreased to a lower set threshold. This grants us some immunity to
 *    noise, as the analog signal rises and lowers on a jagged path when the sensor looks 
 *    over the encoder
 * 3. If the odometer's analog signal is decreased below the lowered threshold, the threshold
 *    is raised and halfWheelCount will be incremented again.
 */
SIGNAL(TIMER0_COMPA_vect){
  uint16_t analog = analogRead(ODOMETER_ANALOG);
  if(threshold == UPPER_THRESHOLD && analog > UPPER_THRESHOLD){
    threshold = LOWER_THRESHOLD;
    halfTurns++;
  }else if(threshold == LOWER_THRESHOLD && analog < LOWER_THRESHOLD){
    threshold = UPPER_THRESHOLD;
    halfTurns++;
  }
  
}

//steerLeft() directs the robot to turn left. It is best to use this rather than
//using digitalWrite() in order to avoid control errors.
void steerLeft(){
  digitalWrite(RTRN, LOW);
  digitalWrite(LTRN, HIGH);
}

//steerRight() directs the robot to turn right. It is best to use this rather than
//using digitalWrite() in order to avoid control errors.
void steerRight(){
  digitalWrite(RTRN, HIGH);
  digitalWrite(LTRN, LOW);
}

//steerStraight() directs the robot to turn neither left nor right. It is best to use this rather than
//using digitalWrite() in order to avoid control errors.
void steerStraight(){
  digitalWrite(RTRN, LOW);
  digitalWrite(LTRN, LOW);
}

//motorForward() directs the throttle forward and updates the throttle variable.
void motorForward(){
  digitalWrite(FWRD, HIGH);
  digitalWrite(BWRD, LOW);
  throttle = FORWARD;
}

//motorBack() directs the throttle backward and updates the throttle variable.
void motorBack(){
  digitalWrite(FWRD, LOW);
  digitalWrite(BWRD, HIGH);
  throttle = REVERSE;
}

//motorForward() directs the throttle to cease and updates the throttle variable.
void motorStop(){
  digitalWrite(FWRD, LOW);
  digitalWrite(BWRD, LOW);
  throttle = STILL;
}

/*
 * killSwitch checks whether or not a kill switch signal has been given. The code will stop if the kill
 * switch has been thrown.
 * 
 * Note: we haven't looked at the actual hookup for the killswitch yet, but are expecting to use a bluetooth
 * connection to implement this when wireless logging is added.
 */
bool killSwitch(){
  return false;
}

/*calibrateMag() carries the robot through the calibration sequence. There are three steps
 * to this process, as the robot must be able to account for any direction of steering. When the
 * robot is in a turn, the steering motor is stalled, causing it to draw more current than a
 * turning motor. This causes it to act like a fairly weak electromagnet. While this isn't
 * enough to cause visible changes, it does interfere with the magnetometer's readings.
 * 
 * The calibration process corrects the error inherent in the magnetometer due to temperature, nearby
 * ferrous metals, and magnetic fields. For the time being, calibration will occur every startup.
 * If it becomes more convenient, we may hard-code calibration values into the robot when it has
 * achieved a more stable state.
 */
void calibrateMag(){
  int16_t ax, ay,az, gx, gy, gz, mx, my, mz; //variables for all three of the IMU's 3-axis sensors
  //Currently, the robot gathers all three forms of data in case they will be used later.
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
  epochTime = millis();
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

  cal_min = (mx_min + my_min + mz_min) / 3;
  cal_max = (mx_max + my_max + mz_max) / 3;
}

/*
 * navigate() uses all of the data at its disposal to find the robot's current heading and position.
 * For heading, it uses magnetometer data and the arctangent function. For position, it multiplies the robot's speed
 * per millisecond (represented by MPS / 1000) by the milliseconds passed since the last call of navigate().
 * 
 * In addition, navigate() logs sensor data for use in debugging.
 */
void navigate(){
  // We may not end up using time intervals, but we can hold on to it for now
  // in case it becomes useful.
  unsigned long lastTime = epochTime;
  epochTime = millis();
  unsigned long intervalTime = epochTime - lastTime;
  
  unsigned long intervalTurns = halfTurns - lastTurns;
  unsigned long lastTurns = halfTurns;
  
  int16_t ax, ay, az, gx, gy, gz, mx, my, mz;
  mag.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
  int16_t mx_adjusted = map(mx - mx_bias, mx_min, mx_max, cal_min, cal_max);
  int16_t my_adjusted = map(my - my_bias, my_min, my_max, cal_min, cal_max);
  heading = atan2(-my_adjusted,mx_adjusted) * 180 / PI - 180;
  while(heading < 0) heading += 360;

  JVector addVector(((float)intervalTurns * WHEEL_CIRC_CM / 200.0) * sin(heading * PI / 180.0),
                    ((float)intervalTurns * WHEEL_CIRC_CM / 200.0) * cos(heading * PI / 180.0));
  
  if(throttle == FORWARD){
    pos += addVector;
  }else if(throttle == REVERSE){
    pos -= addVector;
  }
  
  Serial.print(halfTurns);
  Serial.print(", ");
  Serial.print(digitalRead(ODOMETER_DIGITAL));
  Serial.print(", ");
  Serial.print(analogRead(ODOMETER_ANALOG));
  Serial.print(", ");
  Serial.print(odometerMidpoint);
  Serial.print(", ");
  Serial.print(mx);
  Serial.print(", ");
  Serial.print(my);
  Serial.print(", ");
  Serial.print(mz);
  Serial.print(", ");
  Serial.print(intervalTime);
  Serial.print(", ");
  Serial.print(addVector.getX());
  Serial.print(", ");
  Serial.print(addVector.getY());
  Serial.print(", ");
  Serial.print(pos.getX());
  Serial.print(", ");
  Serial.print(pos.getY());
  Serial.print(", ");
  Serial.print(pos.getZ());
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
  Serial.print(heading);
}


/*
 * guide() takes into account the robot's current position, the next waypoint, and the previous waypoint.
 * It then calculates the necessary change in heading so that control() can react appropriately.
 * In the final version, guide() will also decide whether or not the robot has passed the current waypoint.
 * 
 * In previous iterations and test versions, guide() was solely used to maintain the robot's course or verify
 * its ability to steer in a particular magnetic direction.
 */
void guide(){

  //Have I passed the previous waypoint?

  float dotProduct = (waypoints[wpIndex] - waypoints[wpIndex - 1]).dot(waypoints[wpIndex] - pos);
  if(dotProduct <= 0.0){
    wpIndex++;
  }
  
  //compute headingChange based upon pos and the next waypoint

  if(wpIndex >= WP_LENGTH){
    //If waypoints have all been reached, send an impossible heading change to control
    headingChange = -999.0;
  }else{
    desiredHeading = atan2( waypoints[wpIndex].getX() - pos.getX() , waypoints[wpIndex].getY() - pos.getY()) * 180 / PI;
    while(desiredHeading < 0) desiredHeading += 360;
  
    headingChange = desiredHeading - heading;
    
    if(headingChange > 180) headingChange -= 360;
    if(headingChange < -180) headingChange += 360;
  }
  
  Serial.print(", ");
  Serial.print(desiredHeading);
  Serial.print(", ");
  Serial.println(headingChange);
}

/*
 * control() turns left or right if appropriate, and stops if the robot has reached its final destination.
 */

void control(){
  if(killSwitch()){
    motorStop();
    while(killSwitch()){
      epochTime = millis();
    }
  }else{
    motorForward();
    if(headingChange < -900){
      steerStraight();
      motorStop();
      while(true){
      }
    }else if(abs(headingChange) >= DEADBAND / 2){
      //Time to TURN
      if(headingChange < 0){
        steerLeft();
      }else{
        steerRight();
      }
    }else{
      steerStraight();
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
