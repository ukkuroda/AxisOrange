#include <M5StickC.h>
#include "ImuReader/mahony/MahonyAHRS.h"

float accX = 0.0F;
float accY = 0.0F;
float accZ = 0.0F;

float gyroX = 0.0F;
float gyroY = 0.0F;
float gyroZ = 0.0F;

const float offsetGyroX = -1.76546F;
const float offsetGyroY = -6.8906F;
const float offsetGyroZ = 14.59196F;

float quatW = 1.0F;
float quatX = 0.0F;
float quatY = 0.0F;
float quatZ = 0.0F;

float pitch = 0.0F;
float roll  = 0.0F;
float yaw   = 0.0F;

mahony::MahonyAHRS ahrs;

void setup() {
  // put your setup code here, to run once:
  M5.begin();
  M5.IMU.Init();
  M5.Lcd.setRotation(3);
  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setTextSize(1);
  M5.Lcd.setCursor(40, 0);
  M5.Lcd.println("IMU TEST");
  M5.Lcd.setCursor(0, 10);
  M5.Lcd.println("  X       Y       Z");
  M5.Lcd.setCursor(0, 50);
  M5.Lcd.println("  Pitch   Roll    Yaw");

  Serial.begin(115200);
}

float temp = 0;
/*****************************************
M5.IMU.getGyroData(&gyroX,&gyroY,&gyroZ);
M5.IMU.getAccelData(&accX,&accY,&accZ);
M5.IMU.getAhrsData(&pitch,&roll,&yaw);
M5.IMU.getTempData(&temp);
*****************************************/
void loop() {
  // put your main code here, to run repeatedly:
  M5.IMU.getGyroData(&gyroX,&gyroY,&gyroZ);
  M5.IMU.getAccelData(&accX,&accY,&accZ);
  
  // offset
  gyroX -= offsetGyroX;
  gyroY -= offsetGyroY;
  gyroZ -= offsetGyroZ;

  Serial.printf("%f, %f, %f\n", gyroX, gyroY, gyroZ);

  ahrs.UpdateQuaternion(
    gyroX * DEG_TO_RAD, gyroY * DEG_TO_RAD, gyroZ * DEG_TO_RAD, 
    accX, accY, accZ,
    quatW, quatX, quatY, quatZ);
  ahrs.QuaternionToEuler(
    quatW, quatX, quatY, quatZ,
    pitch, roll, yaw);

  M5.IMU.getTempData(&temp);
  
  M5.Lcd.setCursor(0, 20);
  M5.Lcd.printf("%6.2f  %6.2f  %6.2f      ", gyroX, gyroY, gyroZ);
  M5.Lcd.setCursor(140, 20);
  M5.Lcd.print("o/s");
  M5.Lcd.setCursor(0, 30);
  M5.Lcd.printf(" %5.2f   %5.2f   %5.2f   ", accX, accY, accZ);
  M5.Lcd.setCursor(140, 30);
  M5.Lcd.print("G");
  M5.Lcd.setCursor(0, 60);
  M5.Lcd.printf(" %5.2f   %5.2f   %5.2f   ", pitch, roll, yaw);

  M5.Lcd.setCursor(0, 70);
  M5.Lcd.printf("Temperature : %.2f C", temp);
  delay(1000);
}
