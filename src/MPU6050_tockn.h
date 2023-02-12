#ifndef MPU6050_SLOPED_H
#define MPU6050_SLOPED_H

#include <Arduino.h>
#include "Wire.h"



#define MPU6050_ADDR         0x68
#define MPU6050_SMPLRT_DIV   0x19
#define MPU6050_CONFIG       0x1a
#define MPU6050_GYRO_CONFIG  0x1b
#define MPU6050_ACCEL_CONFIG 0x1c
#define MPU6050_WHO_AM_I     0x75
#define MPU6050_PWR_MGMT_1   0x6b
#define MPU6050_TEMP_H       0x41
#define MPU6050_TEMP_L       0x42

class MPU6050{
  public:

  MPU6050(TwoWire &w);
  MPU6050(TwoWire &w, float aC, float gC);

  void begin();

  void setGyroOffsets(float x, float y, float z);

  void writeMPU6050(byte reg, byte data);
  byte readMPU6050(byte reg);

  int16_t getRawAccX(){ return rawAccX; };
  int16_t getRawAccY(){ return rawAccY; };
  int16_t getRawAccZ(){ return rawAccZ; };

  int16_t getRawTemp(){ return rawTemp; };

  int16_t getRawGyroX(){ return rawGyroX; };
  int16_t getRawGyroY(){ return rawGyroY; };
  int16_t getRawGyroZ(){ return rawGyroZ; };

  float getTemp(){ return temp; };

  float getAccX(){ return accX; };
  float getAccY(){ return accY; };
  float getAccZ(){ return accZ; };

  float getGyroX(){ return gyroX; };
  float getGyroY(){ return gyroY; };
  double getGyroZ(){ return gyroZ; };

	void calibrateGyro(bool console = false, bool evaluate = true, uint16_t delayBefore = 1000, uint16_t delayAfter = 2000);

  float getGyroXoffset(){ return gyroXoffset; };
  float getGyroYoffset(){ return gyroYoffset; };
  double getGyroZoffset(){ return gyroZoffset; };

  void update();

  float getAccAngleX(){ return angleAccX; };
  float getAccAngleY(){ return angleAccY; };

  float getGyroAngleX(){ return angleGyroX; };
  float getGyroAngleY(){ return angleGyroY; };
  double getGyroAngleZ(){ return angleGyroZ; };

  float getAngleX(){ return angleX; };
  float getAngleY(){ return angleY; };
  double getAngleZ(){ return angleZ; };

  double slopempu();

  void setsenstivity(double value)
  {
    senstivity = value;
  }

  double bhaluuZangle();

  void resetparams();


  private:
  void evaluatesensordata();
  TwoWire *wire;

  int16_t rawAccX, rawAccY, rawAccZ, rawTemp,
  rawGyroX, rawGyroY, rawGyroZ;

  float gyroXoffset, gyroYoffset;
  double gyroZoffset;

  float temp, accX, accY, accZ, gyroX, gyroY;
  double gyroZ;

  float angleGyroX, angleGyroY;
  double angleGyroZ;
  float angleAccX, angleAccY, angleAccZ;

  float angleX, angleY;
  double angleZ;

  double interval;
  long preInterval;

  float accCoef, gyroCoef;

//shahab's variables for slopes
  double previous = 0;
  double slope =0;
  unsigned long previoustime = 0;
  int duration = 50;
  double diff =0;
//getrectified parameters
  double start =0;
  double senstivity = 0.15;
  double diff2 =0;
  double angle =0;
};



#endif
