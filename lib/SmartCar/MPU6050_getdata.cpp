/*
 * @Author: ELEGOO
 * @Date: 2019-10-22 11:59:09
 * @LastEditTime: 2020-06-19 11:57:50
 * @LastEditors: Changhua
 * @Description: conqueror robot tank
 * @FilePath: 
 */

#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"
#include "MPU6050_getdata.h"

#include <stdio.h>
#include <math.h>

MPU6050 accelgyro;
MPU6050_getdata MPU6050Getdata;

// static void MsTimer2_MPU6050getdata(void)
// {
//   sei();
//   int16_t ax, ay, az, gx, gy, gz;
//   accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz); //读取六轴原始数值
//   float gyroz = -(gz - MPU6050Getdata.gzo) / 131 * 0.005f;
//   MPU6050Getdata.yaw += gyroz;
// }

bool MPU6050_getdata::MPU6050_dveInit(void)
{
  Wire.begin();
  uint8_t chip_id = 0x00;
  uint8_t cout;
  do
  {
    chip_id = accelgyro.getDeviceID();
    Serial.print("MPU6050_chip_id: ");
    Serial.println(chip_id);
    delay(10);
    cout += 1;
    if (cout > 10)
    {
      return true;
    }
  } while (chip_id == 0X00 || chip_id == 0XFF); //确保从机设备在线（强行等待 获取 ID ）
  accelgyro.initialize();
  // unsigned short times = 100; //采样次数
  // for (int i = 0; i < times; i++)
  // {
  //   gz = accelgyro.getRotationZ();
  //   gzo += gz;
  // }
  // gzo /= times; //计算陀螺仪偏移
  return false;
}
bool MPU6050_getdata::MPU6050_calibration(void)
{
  int times = 200; //采样次数
  for (int i = 0; i < times; i++)
  {
    gz = accelgyro.getRotationZ();
    vx = accelgyro.getAccelerationX();
    vy = accelgyro.getAccelerationY();
    gzo += gz;
    axo += vx;
    ayo += vy;
    delay(5); // Small delay for stable readings
  }
  gzo /= times; //计算陀螺仪偏移
  axo /= times;
  ayo /= times;
  // gzo = accelgyro.getRotationZ();
  return false;
}
bool MPU6050_getdata::MPU6050_dveGetEulerAngles(float *Yaw)
{
  now = millis();   //当前时间(ms)
  dt = (now - lastTime) / 1000.0; //微分时间(s)
  lastTime = now;                 //上一次采样时间(ms)
  gz = accelgyro.getRotationZ();
  float gyroz = -(gz - gzo) / 131.0 * dt; //z轴角速度
  if (fabs(gyroz) < 0.05)
  {
    gyroz = 0.00;
  }
  agz += gyroz; //z轴角速度积分
  *Yaw = agz;
  return false;
}

float MPU6050_getdata::lowPassFilter(float current, float prev, float alpha){
  return alpha * current + ( 1 - alpha ) * prev;
}

float MPU6050_getdata::MPU6050_getDistance(char axis){

  now_dist = millis();
  
  float currAccY = accelgyro.getAccelerationY();
  float currAccX = accelgyro.getAccelerationX();

  while (currAccY > 1200 || -1200 > currAccY){
    currAccY = accelgyro.getAccelerationY();
    now = millis();
  }

  acc_dt = (now_dist - lastTime_dist) / 1000.0; // dt in seconds
  lastTime_dist = now_dist;

  float filteredAccY = lowPassFilter(currAccY, prevAccY, lowPassAlpha);
  float filteredAccX = lowPassFilter(currAccX, prevAccX, lowPassAlpha);

  accX = (filteredAccX - axo) / accScaleFactor * 9.81;
  accY = (filteredAccY - ayo) / accScaleFactor * 9.81;

  prevAccY = currAccY;
  prevAccX = currAccX;

  if (axis == 'y') accX = 0;
  else if (axis == 'x') accY = 0;

  if (fabs(accX) < accThreshold) accX = 0;
  if (fabs(accY) < accThreshold) accY = 0;

  // Serial.println("AccX: " + String(accX,10) + " | AccY: " + String(accY,10));

  vx += accX * acc_dt; // change in velocity
  vy += accY * acc_dt; // change in velocity

  // Serial.println("Vx: " + String(vx,10) + " | Vy: " + String(vy,10));

  distX += fabs(vx * acc_dt); // distX is equivalent of X distance alr traveled
  distY += fabs(vy * acc_dt); // distX is equivalent of Y distance alr traveled

  float Distance = sqrt(pow(distX, 2) + pow(distY, 2));

  Serial.println(
    "Dist (overall): " + String(Distance, 5) + " | AccY: " + String(accY, 5) + " | DistY: "
    + String(distY, 5) + " | VY: " + String(vy, 5) + " | dt: " + String(acc_dt) +
    " | Raw AccY: " + String(currAccY, 5) + " | filtered AccY: " + String(filteredAccY, 5)
  );

  return Distance;
}

void MPU6050_getdata::resetYawAtIntervals() {
  agz = 0; // Reset the integrated yaw angle
  now = millis();
  lastTime = now;
}

void MPU6050_getdata::resetDistance() {
  vx = 0;
  vy = 0;
  distX = 0;
  distY = 0;
  now_dist = millis();
  lastTime_dist = now_dist;
}