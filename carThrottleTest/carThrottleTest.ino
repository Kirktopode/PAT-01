const int LTRN = 7;
const int RTRN = 8;
const int FWRD = 11;
const int BWRD = 12;

void setup() {
  // put your setup code here, to run once:
  pinMode(LTRN, OUTPUT);
  pinMode(RTRN, OUTPUT);
  pinMode(FWRD, OUTPUT);
  pinMode(BWRD, OUTPUT);
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  delay(1000);
  for(int i = 0; i <= 25; i++){
    analogWrite(LTRN, i * 10);
    Serial.println(i * 10);
    delay(1000);
  }
  while(true);
}
