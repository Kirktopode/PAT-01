//leonardo hookup
/*
const int LTRN = 7;
const int RTRN = 8;
const int FWRD = 11;
const int BWRD = 12;
*/

//micro hookup

const int LTRN = 5;
const int RTRN = 16;
const int FWRD = 15;
const int BWRD = 14;
const int BUTTON = 7;

void setup() {
  // put your setup code here, to run once:
  pinMode(LTRN, OUTPUT);
  pinMode(RTRN, OUTPUT);
  pinMode(FWRD, OUTPUT);
  pinMode(BWRD, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  delay(1000);
  digitalWrite(FWRD, HIGH);
  delay(1000);
  digitalWrite(LTRN, HIGH);
  delay(1000);
  digitalWrite(LTRN, LOW);
  digitalWrite(RTRN, HIGH);
  delay(1000);
  digitalWrite(LTRN, LOW);
  digitalWrite(RTRN, LOW);
  digitalWrite(FWRD, LOW);
  delay(500);
  digitalWrite(BWRD, HIGH);
  delay(1000);
  digitalWrite(BWRD, LOW);
  while(true);
}
