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
  bool MPU6050_getDistance(float *Distance);
  void resetDistance();
  void resetYawAtIntervals();

public:
  //int16_t ax, ay, az, gx, gy, gz;
  int16_t gz, vx, vy;
  //float pith, roll, yaw;
  unsigned long now, lastTime = 0;
  unsigned long now_dist, lastTime_dist = 0;
  float dt, vdt;      //微分时间
  float agz = 0, avx = 0, avy = 0; //角度变量
  long gzo = 0, vxo = 0, vyo = 0;  //陀螺仪偏移量
};

extern MPU6050_getdata MPU6050Getdata;
#endif