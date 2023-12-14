#include <avr/wdt.h>
#include "DeviceDriverSet_xxx0.h"
#include "ApplicationFunctionSet_xxx0.cpp"
#include <Arduino.h>
DeviceDriverSet_Motor AppMotor;
Application_xxx Application_ConquerorCarxxx0;
MPU6050_getdata AppMPU6050getdata;
ConquerorCarMotionControl status = Forward;

class SmartCar
{

private:
public:
    void init()
    {
        Serial.println("Initiated SmartCar");
        AppMotor.DeviceDriverSet_Motor_Init();
        Serial.println("Initiated Motor Driver");
        AppMPU6050getdata.MPU6050_dveInit();
        Serial.println("Initiated MPU Driver");
        delay(2000);
        AppMPU6050getdata.MPU6050_calibration();
        Serial.println("Done SmartCar");
    }

    void moveFoward(int speed)
    {
        ApplicationFunctionSet_ConquerorCarMotionControl(
            ConquerorCarMotionControl::Forward,
            speed // 0-255
        );
    }

    void moveBackward(int speed)
    {
        ApplicationFunctionSet_ConquerorCarMotionControl(
            ConquerorCarMotionControl::Backward,
            speed // 0-255
        );
    }

    void moveLeft(int speed)
    {
        ApplicationFunctionSet_ConquerorCarMotionControl(
            ConquerorCarMotionControl::Left,
            speed // 0-255
        );
    }

    void moveRight(int speed)
    {
        ApplicationFunctionSet_ConquerorCarMotionControl(
            ConquerorCarMotionControl::Right,
            speed // 0-255
        );
    }

    void stop()
    {
        ApplicationFunctionSet_ConquerorCarMotionControl(
            ConquerorCarMotionControl::stop_it,
            0);
    }
};