#include <NewPing.h>

#define trigPinCenter 7
#define echoPinCenter 6
#define trigPinLeft 5
#define echoPinLeft 4
#define trigPinRight 3
#define echoPinRight 2
#define MAX_DISTANCE 100
#define SONAR_NUM 3

NewPing sonar[3] = {
  NewPing(trigPinLeft, echoPinLeft, MAX_DISTANCE), // max distance is 200cm.
  NewPing(trigPinCenter, echoPinCenter, MAX_DISTANCE),
  NewPing(trigPinRight, echoPinRight, MAX_DISTANCE)
};
  
void setup() {
  pinMode(trigPinLeft,OUTPUT); // Trigger is an output pin
  pinMode(trigPinRight,OUTPUT); // Trigger is an output pin
  pinMode(trigPinCenter, OUTPUT);
  pinMode(echoPinLeft,INPUT); // Echo is an input pin
  pinMode(echoPinRight,INPUT); // Echo is an input pin
  pinMode(echoPinCenter, INPUT);
  // put your setup code here, to run once:
  Serial.begin(115200);

}

void loop() {
  // put your main code here, to run repeatedly:
   for (uint8_t i = 0; i < SONAR_NUM; i++) { // Loop through each sensor and display results. "uint8_t" is the same type as "byte".
    delay(50); // Wait 50ms between pings (about 20 pings/sec). 29ms should be the shortest delay between pings.
    Serial.print(i);
    Serial.print("=");
    Serial.print(sonar[i].ping_cm());
    Serial.print("cm ");
    // if the left has 30 centimeters and right is clear than turn right.
    // if the center has 30 centimeters and left has 30 centimers turn right

    // If the center left and right all all have 30 centimeters turn 90 degrees.

    // If the right or right and center both have 30 centimeters turn left.
  }
  Serial.println();

  
}
