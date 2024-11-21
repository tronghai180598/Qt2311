#include "Arduino.h"
#include "Servo.h"
#include <Wire.h>

Servo Motor1;
Servo Motor2;
const int MPU = 0x68;
#define right_motor 3
#define left_motor 9
const int Umn = 1000;
const int Umx = 2000;

int16_t AccX, AccY, AccZ, GyroX, GyroY, GyroZ;
int16_t bTr[2];
int16_t bRc[2];
int saturate(int val, int mn, int mx){ return (val < mn) ? mn : ( (val > mx)? mx :val ); }

void setup() {
  Serial.begin(115200);
  Wire.begin();
  
  // Initialize MPU6050
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);
  Wire.write(0x00);  // Wake up MPU6050
  Wire.endTransmission(true);

  Motor1.attach(right_motor);
  Motor2.attach(left_motor);
  delay(2);
  Motor1.write(Umn);
  Motor2.write(Umn);
  delay(2000);
  Motor1.write(Umx);
  Motor2.write(Umx);
  delay(2);
  Motor1.write(Umn);
  Motor2.write(Umn);
}

void loop() {
  // Read accelerometer data
  Wire.beginTransmission(MPU);
  Wire.write(0x3B); // Starting register for accelerometer
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true);
  AccX = (Wire.read() << 8 | Wire.read());
  AccY = (Wire.read() << 8 | Wire.read());
  AccZ = (Wire.read() << 8 | Wire.read());

  // Read gyroscope data
  Wire.beginTransmission(MPU);
  Wire.write(0x43); // Starting register for gyroscope
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true);
  GyroX = (Wire.read() << 8 | Wire.read());
  GyroY = (Wire.read() << 8 | Wire.read());
  GyroZ = (Wire.read() << 8 | Wire.read());

  // Send sensor data to Qt app as a comma-separated string
  bTr[0] = AccY;
  bTr[1] = GyroX;
  
  Serial.write((const char*) bTr, 4);
  uint16_t cnt = 1000;
  while(cnt--){
    if(Serial.available() ){
      Serial.readBytes((char*) bRc, 4);      
      Motor1.write(saturate(bRc[0], Umn, Umx));
      Motor2.write(saturate(bRc[1], Umn, Umx));
      break;
    }
    else {
      delayMicroseconds(100);
    }
  }
}
