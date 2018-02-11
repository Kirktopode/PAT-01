
int byteReceived;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);  // default baud rate for the HC-05 module
  Serial.print("Type in Box above.");
  Serial.println();
  
}

void loop() {
  // put your main code here, to run repeatedly:

  if(Serial.available() > 0)
    {
    byteReceived = Serial.read();
    Serial.print("Decimal value: ");
    Serial.print(byteReceived);
    Serial.println();
    Serial.print("Hex value: ");
    Serial.print(byteReceived, HEX);
    Serial.println();
    Serial.print("Char value: ");
    Serial.print(char(byteReceived));
    Serial.println();
      
    
    }
}
