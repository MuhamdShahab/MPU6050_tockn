#include "MPU6050_tockn.h"
#include "arduino.h"

MPU6050::MPU6050(TwoWire &w){
  wire = &w;
  accCoef = 0.02f;
  gyroCoef = 0.98f;
}

MPU6050::MPU6050(TwoWire &w, float aC, float gC){
  wire = &w;
  accCoef = aC;
  gyroCoef = gC;
}

void MPU6050::begin(){
  writeMPU6050(MPU6050_SMPLRT_DIV, 0x00);
  writeMPU6050(MPU6050_CONFIG, 0x00);
  writeMPU6050(MPU6050_GYRO_CONFIG, 0x08);
  writeMPU6050(MPU6050_ACCEL_CONFIG, 0x00);
  writeMPU6050(MPU6050_PWR_MGMT_1, 0x01);
  this->update();
  angleGyroX = 0;
  angleGyroY = 0;
  angleX = this->getAccAngleX();
  angleY = this->getAccAngleY();
  preInterval = millis();
}

void MPU6050::writeMPU6050(byte reg, byte data){
  wire->beginTransmission(MPU6050_ADDR);
  wire->write(reg);
  wire->write(data);
  wire->endTransmission();
}

byte MPU6050::readMPU6050(byte reg) {
  wire->beginTransmission(MPU6050_ADDR);
  wire->write(reg);
  wire->endTransmission(true);
  wire->requestFrom(MPU6050_ADDR, 1);
  byte data =  wire->read();
  return data;
}

void MPU6050::setGyroOffsets(float x, float y, float z){
  gyroXoffset = x;
  gyroYoffset = y;
  gyroZoffset = z;
}

//Sometime the Z angle keeps on increasing with high magnitude, in that below statement screens such behaivios and reads the data again
void MPU6050::evaluatesensordata()
{
  bhaluuZangle();
  delay(300);
  double prevangle = bhaluuZangle();
  Serial.println(prevangle);
  delay(300);
  Serial.println(bhaluuZangle());
  if(bhaluuZangle() != prevangle)
  {
    Serial.println("Issues Found ! Repeating Process.");
    Serial.println("========================================");
    resetparams();
    calibrateGyro();
  }
  else
  {
    update();
    delay(200);
    start = getAngleZ();
    Serial.println("Pre-Builts Test Passed.");
    Serial.println("========================================");
  }
}
void MPU6050::calibrateGyro(bool console, bool evaluate, uint16_t delayBefore, uint16_t delayAfter){
	float x = 0, y = 0, z = 0;
	int16_t rx, ry, rz;

  delay(delayBefore);
	if(console){
    Serial.println();
    Serial.println("========================================");
    Serial.println("Calculating gyro offsets");
    Serial.println("DO NOT MOVE MPU6050");
  }
  for(int i = 0; i < 3000; i++){
    if(console && i % 1000 == 0){
      Serial.print(".");
    }
    wire->beginTransmission(MPU6050_ADDR);
    wire->write(0x43);
    wire->endTransmission(false);
    wire->requestFrom((int)MPU6050_ADDR, 6);

    rx = wire->read() << 8 | wire->read();
    ry = wire->read() << 8 | wire->read();
    rz = wire->read() << 8 | wire->read();

    x += ((float)rx) / 65.5;
    y += ((float)ry) / 65.5;
    z += ((float)rz) / 65.5;
  }
  gyroXoffset = x / 3000;
  gyroYoffset = y / 3000;
  gyroZoffset = z / 3000;
  if(gyroXoffset == gyroYoffset && gyroYoffset == gyroZoffset)
  {
    calibrateGyro();
  }
  else
  {
    if(console)
    {
    Serial.println("Done!");
    Serial.print("X : ");Serial.println(gyroXoffset);
    Serial.print("Y : ");Serial.println(gyroYoffset);
    Serial.print("Z : ");Serial.println(gyroZoffset);
    Serial.println("Evaluating Sensor's Performance.");
		delay(delayAfter);
    }
    else
    {
      //pass
    }
	}

  
  if(evaluate)
   evaluatesensordata();
  else  
    {
      update();
      start = getAngleZ();
      Serial.println("========================================");
      delay(1000);
    }

}

void MPU6050::update(){
	wire->beginTransmission(MPU6050_ADDR);
	wire->write(0x3B);
	wire->endTransmission(false);
	wire->requestFrom((int)MPU6050_ADDR, 14);

  rawAccX = wire->read() << 8 | wire->read();
  rawAccY = wire->read() << 8 | wire->read();
  rawAccZ = wire->read() << 8 | wire->read();
  rawTemp = wire->read() << 8 | wire->read();
  rawGyroX = wire->read() << 8 | wire->read();
  rawGyroY = wire->read() << 8 | wire->read();
  rawGyroZ = wire->read() << 8 | wire->read();

  temp = (rawTemp + 12412.0) / 340.0;

  accX = ((float)rawAccX) / 16384.0;
  accY = ((float)rawAccY) / 16384.0;
  accZ = ((float)rawAccZ) / 16384.0;

  angleAccX = atan2(accY, accZ + abs(accX)) * 360 / 2.0 / PI;
  angleAccY = atan2(accX, accZ + abs(accY)) * 360 / -2.0 / PI;
 
  gyroX = ((float)rawGyroX) / 65.5;
  gyroY = ((float)rawGyroY) / 65.5;
  gyroZ = ((double)rawGyroZ) / 65.5;

  gyroX -= gyroXoffset;
  gyroY -= gyroYoffset;
  gyroZ -= gyroZoffset;

  interval = (millis() - preInterval) * 0.001;

  angleGyroX += gyroX * interval;
  angleGyroY += gyroY * interval;
  angleGyroZ += gyroZ *interval;

  angleX = (gyroCoef * (angleX + gyroX * interval)) + (accCoef * angleAccX);
  angleY = (gyroCoef * (angleY + gyroY * interval)) + (accCoef * angleAccY);
  angleZ = angleGyroZ;

  preInterval = millis();

}

/*Slope of the accumulating noise is found using the derivate
when the sensor is displaced the slope is different as compared to the previoous
so it records the values as the genuince angles.*/
double MPU6050::slopempu()
{
  update();
  if((millis() - previoustime) >= duration)
  {
    diff = getAngleZ() - previous;
    slope = ((diff*1000)/duration);
    //Serial.print("Slope : ");Serial.println(slope);
    previoustime = millis();
    previous = getAngleZ();
  }
  else
  {}
  return slope;
}
//part of above statements
double MPU6050::bhaluuZangle()
{
  diff2 = getAngleZ()- start;
  if((slopempu() > senstivity) || (slopempu() < -senstivity))
  {
    angle = getAngleZ() - diff2;
    start = getAngleZ();
    return angle;
  } 
 else{
   return angle;
 }

}

void MPU6050::resetparams()
  {
    start = 0;
    previous = 0;
    slope =0;
    diff =0;
    diff2 = 0;
    angle = 0;
  }
