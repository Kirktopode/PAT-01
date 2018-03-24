#include <NewPing.h>

#define trigPinLeft 2
#define echoPinLeft 3
#define trigPinRight 4
#define echoPinRight 5
NewPing sonar[2] = {
  NewPing(trigPinLeft, echoPinLeft, 200), // max distance is 200cm.
  NewPing(trigPinRight, echoPinRight, 200)
};
  
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

}

void loop() {
  // put your main code here, to run repeatedly:
  delay(50);
  int uS = sonar[0].ping_cm();
  Serial.print("Left Sonar: ");
  Serial.println(uS);
  uS = sonar[1].ping_cm();
  Serial.print("Right Sonar: ");
  Serial.println(uS);
}
