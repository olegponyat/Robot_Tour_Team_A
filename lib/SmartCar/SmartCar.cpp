#include <avr/wdt.h>
#include "DeviceDriverSet_xxx0.h"
#include "ApplicationFunctionSet_xxx0.cpp"
#include <Arduino.h>
#include <system.cpp>
DeviceDriverSet_Motor AppMotor;
Application_xxx Application_ConquerorCarxxx0;
MPU6050_getdata AppMPU6050getdata;

bool debug = true;

class SmartCar
{

private:
    // Call this after a turn is completed
    void updateYawReference()
    {
        AppMPU6050getdata.MPU6050_dveGetEulerAngles(&Yaw);
        yaw_So = Yaw; // Set the new reference yaw
    }

    int calculateGradualSpeed(float currentYaw, float targetYaw, int initialSpeed){
        // Calculate the remaining angle to turn
        float remainingAngle = abs(currentYaw - targetYaw);

        // Gradually reduce speed based on the remaining angle
        int speed = initialSpeed * (remainingAngle / 90); // Assuming 90 is the max turn angle
        if (speed < 50) speed = 50; // Enforce a minimum speed of 50

        // Turn logic
        return speed;
    }

public:
    void init()
    {
        // Serial.begin(9600);
        AppMotor.DeviceDriverSet_Motor_Init();
        AppMPU6050getdata.MPU6050_dveInit();
        delay(2000);
        AppMPU6050getdata.MPU6050_calibration();
    }

    void moveFoward(int speed)
    {
        updateYawReference();
        ApplicationFunctionSet_ConquerorCarMotionControl(
            ConquerorCarMotionControl::Forward,
            speed // 0-255
        );
    }

    void moveBackward(int speed)
    {
        updateYawReference();
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

    void turnLeft(int speed)
    {
        AppMPU6050getdata.MPU6050_dveGetEulerAngles(&Yaw);
        float target = Yaw - 90;

        while (Yaw > target)
        {
            if (debug)
                Serial.println("Current Yaw: " + String(Yaw) + " | target: " + String(target));
            AppMPU6050getdata.MPU6050_dveGetEulerAngles(&Yaw);
            this->moveLeft(calculateGradualSpeed(Yaw, target, speed));
        }

        this->stop();
        updateYawReference();
    }

    void moveRight(int speed)
    {
        ApplicationFunctionSet_ConquerorCarMotionControl(
            ConquerorCarMotionControl::Right,
            speed // 0-255
        );
    }

    void turnRight(int speed)
    {
        AppMPU6050getdata.MPU6050_dveGetEulerAngles(&Yaw);
        float target = Yaw + 90;

        while (Yaw < target)
        {
            if (debug)
                Serial.println("Current Yaw: " + String(Yaw) + " | target: " + String(target));
            AppMPU6050getdata.MPU6050_dveGetEulerAngles(&Yaw);
            this->moveRight(calculateGradualSpeed(Yaw, target, speed));
        }

        this->stop();
        updateYawReference();
    }

    void stop()
    {
        ApplicationFunctionSet_ConquerorCarMotionControl(
            ConquerorCarMotionControl::stop_it,
            0);
    }
};