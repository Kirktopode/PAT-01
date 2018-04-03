//turnrad r 44cm l 38cm

#include "Wire.h" //Used to communicate to the IMU (Accelerometer, gyroscope and magnetometer)
#include "I2Cdev.h" //Helps with the above
#include "MPU9250.h" //Contains a library of functions that streamline communication with the IMU

#include <math.h> //For math (trig functions)
#include <JVector.h> //home-made library for basic 3D vectors. We only use 2 dimensions of it.
#include <NewPing.h>

const int LTRN = 3; //Left turn pin
const int RTRN = A6; //Right turn pin --if both turn pins are HIGH, no turn is made.
const int FWRD = 4; //Forward throttle pin
const int BWRD = 11; //Reverse throttle pin --if both pins are HIGH, no turn is made.
const int BUTTON = 12; //Button pin --for various steps forward in the robot's state
const int ODOMETER_DIGITAL = 10; //Pin to be used to attach the interrupt for the odometer.
const int ODOMETER_ANALOG = A0;

const int WP_DIST = 2; //The distance out to the waypoint in meters

const int TURN_DIST = 70; // Distance at which the robot will turn in centimeters

MPU9250 mpu; //Object used to communicate with the IMU
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
  mpu.initialize();

  Serial.begin(9600); //We initiate Serial communication for logging purposes.
}


const int WP_LENGTH = 2; //Length of the list of waypoints.
const float DEADBAND = 15; //degree range for acceptable forward heading.
//We use a deadband because the steering is only capable of three settings, unlike hobby-grade cars.
const uint16_t UPPER_THRESHOLD = 500; //thresholds used for the odometer
const uint16_t LOWER_THRESHOLD = 200;

const float WHEEL_CIRC_CM = 4.49 * PI;

unsigned long odometerTime = 0;//used for odometer timing
uint16_t odometerMidpoint = 0; //used to identify the midpoint on the odometer

float heading = 0.0, desiredHeading, headingChange;
int wpIndex = 1; //current index in the waypoints array
unsigned long epochTime; //stored in milliseconds and changed with button presses.
unsigned long halfTurns = 0; //updated via interrupt
unsigned long lastTurns = 0; //holds the previous value for navigate updates
uint16_t threshold = UPPER_THRESHOLD;
ThrottleState throttle = STILL; //current throttle state, beginning at STILL
JVector pos(0,0); //current location, beginning at the origin.

//array of waypoints that the robot will travel to. This is not fully implemented.
JVector waypoints[] = {JVector(0,0), JVector(0, 2)};

const int TRIG_LEFT = 9;
const int ECHO_LEFT = 8;
const int TRIG_CENTER = 7;
const int ECHO_CENTER = 6;
const int TRIG_RIGHT = 5;
const int ECHO_RIGHT = 2;
const int MAX_DIST = 100;

NewPing sonarLeft(TRIG_LEFT, ECHO_LEFT, MAX_DIST); //trig pin, echo pin
NewPing sonarCenter(TRIG_CENTER, ECHO_CENTER, MAX_DIST);
NewPing sonarRight(TRIG_RIGHT, ECHO_RIGHT, MAX_DIST);

float leftDist;
float centerDist;
float rightDist;



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

bool received(char c){
  if(Serial.available() > 0){
    char r = Serial.read();
    Serial.print("Received: ");
    Serial.println(r);
    return r == c;
  }
  else{
    return false;
  }
}

/*
 * killSwitch checks whether or not a kill switch signal has been given. The code will stop if the kill
 * switch has been thrown.
 * 
 */
bool killSwitch(){
  return received('k');
}

bool restartSwitch(){
  return received('r');
}

int16_t gz_offset;
const float ROTATION_CONSTANT = 131.2869444;

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
void calibratempu(){
  int16_t ax, ay,az, gx, gy, gz, mx, my, mz; //variables for all three of the IMU's 3-axis sensors
  //Currently, the robot gathers all three forms of data in case they will be used later.
  long gz_total = 0;
  int gz_count = 0;
  while(digitalRead(BUTTON) == LOW){
    mpu.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
    gz_total += gz;
    gz_count++;
    gz_offset = gz_total / gz_count;
    Serial.print(gz);
    Serial.print(", ");
    Serial.print(gz_offset);
    Serial.print(", ");
    Serial.print(gz_total);
    Serial.print(", ");
    Serial.println(gz_count);
  }
  delay(50);
  steerStraight();
  epochTime = millis();
}


const int V_BUFF_LEN = 100;
int v_index = 0;
float velocity = 0.0;
int turns[V_BUFF_LEN];
int ms[V_BUFF_LEN];

int getVelocity(int t, long m){
  if(v_index == V_BUFF_LEN){
    v_index = 0;
    int turns_total = 0;
    int ms_total = 0;
    for(int i = 0; i < V_BUFF_LEN; i++){
      turns_total += turns[i];
      ms_total += ms[i];
    }
    turns[v_index] = turns;
    ms[v_index] = ms;
    v_index++;
    velocity = (float(turns_total) * (WHEEL_CIRC_CM / 2.0)) / (float(ms_total) / 100000);
    return velocity;
  }
  else{
    turns[v_index] = t;
    ms[v_index] = m;
    v_index++;
    return velocity;
  }
}

/*
 * displayMetrics() takes velocity and heading (usually within navigate() ) and displays this information,
 * as well as the globally available object detection information, and displays it on the LCD screen.
 */

void displayMetrics(float velocity, float heading){
  bool avoid = centerDist < TURN_DIST && centerDist != 0;

  //JORDAN'S CODE HERE////////////////////////////////////////////////////////////////
  
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
  int intervalTime = epochTime - lastTime;

  int intervalTurns = halfTurns - lastTurns;
  lastTurns = halfTurns;

  float v = getVelocity(intervalTurns, intervalTime);

  
  int16_t ax, ay, az, gx, gy, gz, mx, my, mz;
  mpu.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);

  long gz_corrected = gz - gz_offset;
  
  
  heading -= (intervalTime * gz_corrected / (1000.0 * ROTATION_CONSTANT));
  while(heading < 0) heading += 360;
  while(heading > 360) heading -= 360;
  JVector addVector(((float)intervalTurns * WHEEL_CIRC_CM / 200.0) * sin(heading * PI / 180.0),
                    ((float)intervalTurns * WHEEL_CIRC_CM / 200.0) * cos(heading * PI / 180.0));
  
  if(throttle == FORWARD){
    pos += addVector;
  }else if(throttle == REVERSE){
    pos -= addVector;
  }

  displayMetrics(v, heading);
  
  //Serial.print(analogRead(ODOMETER_ANALOG));
  //Serial.print(", "); 
  //Serial.print(gz);
  //Serial.print(", ");
  Serial.print(gz_corrected);
  Serial.print(", ");
  Serial.print(intervalTime);
  Serial.print(", ");
  //Serial.print(pos.getX());
  //Serial.print(", ");
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
 * detect() operates the sonar rangefinders and fills values for leftDist, centerDist and 
 * rightDist. Distances are calculated in centimeters.
 */

void detect(){
  leftDist = sonarLeft.ping();
  centerDist = sonarCenter.ping();
  rightDist = sonarRight.ping();
  leftDist /= US_ROUNDTRIP_CM;
  centerDist /= US_ROUNDTRIP_CM;
  rightDist /= US_ROUNDTRIP_CM;
}

/*
 * control() turns left or right if appropriate, and stops if the robot has reached its final destination.
 * It also handles object avoidance logic since the advent of the detect() function.
 */

void control(){
  if(killSwitch()){
    motorStop();
    Serial.println("Killed");
    while(!restartSwitch()){
      epochTime = millis();
    }
    Serial.println("Restart");
  }else{
    motorForward();
    if(headingChange < -900){
      steerStraight();
      motorStop();
      while(true){
        if(digitalRead(BUTTON) == true){
          pos = JVector(0,0);
          wpIndex = 1;
          while(digitalRead(BUTTON) == true){
            delay(500);
          }
          break;
        }
      }
    }else if(centerDist < TURN_DIST && centerDist != 0){
      if(rightDist < TURN_DIST && rightDist != 0){
        if(leftDist < TURN_DIST && leftDist != 0){
          motorBack();
          steerStraight();
        }else{
          motorForward();
          steerLeft();
        }
      }else if(leftDist < TURN_DIST && leftDist != 0){
        motorForward();
        steerRight();
      }
      else{
        motorForward();
        steerLeft();
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
  calibratempu();
  delay(1000); //give me a second to move my hand
  while(true){
    navigate();
    guide();
    detect();
    control();
  }
}
