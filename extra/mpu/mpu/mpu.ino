#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"

MPU6050 mpu;
int16_t ax, ay, az;
int16_t gx, gy, gz;

int valX;
int prevValX;
int valY;
int prevValY;
int incomingByte = 0;

int valX2;
int valY2;

int ledMpu = 8;
int statusCon = 0;

void setup()
{
  Wire.begin();
  Serial.begin(38400);
  pinMode(ledMpu, OUTPUT);
  Serial.println("Initialize MPU");
  mpu.initialize();
  digitalWrite(ledMpu, HIGH);
}

void loop()
{
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  valX = map(ax, 17000, -17000, 0, 100);
  valY = map(ay, -17000, 17000, 0, 100);

  Serial.print("aX: ");
  Serial.print(valX);
  Serial.print(", ");
  Serial.print("aY: ");
  Serial.println(valY);
}
