#include <NewPing.h>

const int trigPin0 = 9;
const int echoPin0 = 5;

const int trigPin1 = 8;
const int echoPin1 = 4;

NewPing sonar[2] = {
  NewPing(trigPin0, echoPin0, 200), // max distance is 200cm.
  NewPing(trigPin1, echoPin1, 200)
};

  
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

}

void loop() {
  // put your main code here, to run repeatedly:
  delay(50);
  int uS = sonar[0].ping();
  Serial.print("Sonar0: ");
  Serial.print(uS);
  uS = sonar[1].ping();
  Serial.print("  Sonar1: ");
  Serial.println(uS);
}
