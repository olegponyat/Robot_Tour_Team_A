/*
 * @Author: ELEGOO
 * @Date: 2019-10-22 11:59:09
 * @LastEditTime: 2020-06-12 17:22:13
 * @LastEditors: Changhua
 * @Description: conqueror robot tank
 * @FilePath: 
 */
#ifndef _MPU6050_getdata_H_
#define _MPU6050_getdata_H_
#include <Arduino.h>
class MPU6050_getdata
{
public:
  bool MPU6050_dveInit(void);
  bool MPU6050_calibration(void);
  bool MPU6050_dveGetEulerAngles(float *Yaw);
  float lowPassFilter(float current, float prev, float alpha);
  float MPU6050_getDistance(char axis);
  void resetDistance();
  void resetYawAtIntervals();

public:
  //int16_t ax, ay, az, gx, gy, gz;
  float gz;
  //float pith, roll, yaw;
  unsigned long now, lastTime = 0;
  unsigned long now_dist = 0, lastTime_dist = 0;
  float dt, acc_dt;      //微分时间
  float agz = 0; //角度变量
  long gzo = 0; //陀螺仪偏移量
  float axo = 0, ayo = 0, accX=0, accY=0;  

  const float accScaleFactor = 16384.0;
  float vx=0, vy=0, distX=0, distY=0;
  float accThreshold = 0.1;
  float lowPassAlpha = 0.05, prevAccY = 0, prevAccX = 0;
  // accuracy
  // 0.05 = 4/5
  // 0.03 = 4.5
  // 0.025 = 4.5/5
  // 0.0125 = 2/5 inconsistent
  // 0.01 = 3.5/5
  // 0 = 4.5/5
};

extern MPU6050_getdata MPU6050Getdata;
#endif