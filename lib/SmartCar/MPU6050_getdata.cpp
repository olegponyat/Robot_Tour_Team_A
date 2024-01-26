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
#include "MPU6050_6Axis_MotionApps20.h"
#include <stdio.h>
#include <math.h>

#include <StandardCplusplus.h>
#include <algorithm>

// MPU6050 accelgyro;
MPU6050_6Axis_MotionApps20 accelgyro;
MPU6050_getdata MPU6050Getdata;

const bool useDMP = true;
Quaternion q;
VectorFloat gravity;
float ypr[3];

// Buffer for reading from the DMP
uint8_t mpuIntStatus;
uint8_t devStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];

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
  if (useDMP) {
    accelgyro.dmpInitialize();
    accelgyro.setDMPEnabled(true);
    packetSize = accelgyro.dmpGetFIFOPacketSize();
    Serial.println("Successfully initiated DMP.");
  }
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
  int times = 50; //采样次数
  float data_points[times];
  for (int i = 0; i < times; i++)
  {
    gz = accelgyro.getRotationZ();
    vy = accelgyro.getAccelerationY();
    gzo += gz;
    ayMean += vy;
    data_points[i] = vy;
    delay(10); // Small delay for stable readings
  }
  gzo /= times; //计算陀螺仪偏移
  ayMean /= times; // mean of acceleration of y
  
  float sum = 0;

  for (int i = 0; i < times; i ++){
    sum += pow(ayMean - data_points[i],2);
  }

  std::sort(data_points, data_points + times);
  stdv = sqrt(sum/times);
  if (times % 2 == 0) {
      // For even number of elements, average the two middle elements
      ayMedian = (data_points[times / 2 - 1] + data_points[times / 2]) / 2.0;
  } else {
      // For odd number of elements, take the middle element
      ayMedian = data_points[times / 2];
  }
  ayo = ayMedian;
  Serial.println("AyMean: " + String(ayMean, 5));
  Serial.println("AyMedian: " + String(ayMedian, 5));
  Serial.println("AyStdv: " + String(stdv, 5));
  Serial.println("(Lower Range) " + String(ayo - (stdv * dev_threshold), 3) + " (Upper Range) " + String(ayo + (stdv * dev_threshold), 3));
  // gzo = accelgyro.getRotationZ();
  return false;
}
bool MPU6050_getdata::MPU6050_dveGetEulerAngles(float *Yaw)
{
  if (useDMP){
    accelgyro.dmpGetCurrentFIFOPacket(fifoBuffer);
    accelgyro.dmpGetQuaternion(&q, fifoBuffer);
    accelgyro.dmpGetGravity(&gravity, &q);
    accelgyro.dmpGetYawPitchRoll(ypr, &q, &gravity);
    *Yaw = ypr[0] * 180.0 / PI;
    return false;
  }

  now = millis();   //当前时间(ms)
  dt = (now - lastTime) / 1000.0; //微分时间(s)
  lastTime = now;                 //上一次采样时间(ms)
  gz = accelgyro.getRotationZ();
  float gyroz = -(gz - gzo) / 131.0 * dt; //z轴角速度
  if (fabs(gyroz) < 0.02)
  {
    gyroz = 0.00;
  }
  agz += gyroz; //z轴角速度积分
  *Yaw = agz;
  return false;
}

bool MPU6050_getdata::invalidValue(float value){
  bool outlier = fabs(value - ayMean) > (stdv * dev_threshold);
  bool change_too_fast = fabs(value - prevAccY) > dev_threshold * stdv;
  return outlier && change_too_fast;
};

float MPU6050_getdata::lowPassFilter(float current, float prev, float alpha){
  return alpha * current + ( 1 - alpha ) * prev;
}

float MPU6050_getdata::MPU6050_getDistance(char axis){

  now_dist = millis();
  float currAccY = accelgyro.getAccelerationY();

  while (invalidValue(currAccY)){
    Serial.println("  Rejected: " + String(currAccY, 5));
    currAccY = accelgyro.getAccelerationY();
    now_dist = millis();
  }

  acc_dt = (now_dist - lastTime_dist) / 1000.0; // dt in seconds
  lastTime_dist = now_dist;

  float filteredAccY = lowPassFilter(currAccY, prevAccY, lowPassAlpha);

  accY = (filteredAccY - ayo) / accScaleFactor * 9.81;

  prevAccY = currAccY;

  // Serial.println("AccY: " + String(accY,10));

  vy += accY * acc_dt; // change in velocity

  distY += fabs(vy * acc_dt); // distX is equivalent of Y distance alr traveled

  float Distance = fabs(distY);

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
  vy = 0;
  distY = 0;
  now_dist = millis();
  lastTime_dist = now_dist;
}