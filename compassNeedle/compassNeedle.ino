
//Connection pins provided in the diagram at the beginning of video

// library provided and code based on: https://github.com/Seeed-Studio/IMU_10DOF
#include "Wire.h"
#include "I2Cdev.h"
#include "MPU9250.h"

// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
MPU9250 accelgyro;
I2Cdev   I2C_M;

int16_t ax, ay, az;
int16_t gx, gy, gz;
int16_t mx, my, mz;

void setup()
{
  Wire.begin();
  Serial.begin(9600);
  Serial.println("Initializing I2C devices...");
  accelgyro.initialize();
  Serial.println("Testing device connections...");
  Serial.println(accelgyro.testConnection() ? "MPU9250 connection successful" : "MPU9250 connection failed");
  delay(1000);
//  Mouse.begin();
}

void loop()
{
  accelgyro.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
  mx = map(mx + 2.5, -62.5, 62.5, -75, 75); 
  my = map(my - 17.5, -66.5, 66.5, -75, 75);

  float heading = atan2( my, mx );
  heading = heading * 180 / PI;
  //heading += 180;
  if(heading < 0) heading += 360;
  
  Serial.println(heading);
  delay(10);
}

