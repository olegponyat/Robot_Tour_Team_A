#include <avr/wdt.h>
#include "DeviceDriverSet_xxx0.h"
#include "ApplicationFunctionSet_xxx0.cpp"
#include <Arduino.h>
#include <system.cpp>
DeviceDriverSet_Motor AppMotor;
Application_xxx Application_ConquerorCarxxx0;
MPU6050_getdata AppMPU6050getdata;

bool debug = false;
const float adjust_threshold = 2.5;
// when the analog is above 100 - set it to 1.o
// when analog is below 100 - set it to higher values such as 2.0 or 2.5
const float moveDelayInterval = 5; // in ms
// 1.0 deg (best so far)
const int lowest_speed = 45;

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
        if (speed < lowest_speed) speed = lowest_speed; // Enforce a minimum speed of 50

        // Turn logic
        return speed;
    }

    int calculateStraightGraduateSpeed(float currentIteration, float totalIteration, int maxSpeed){

        int graduateSpeed = ( 1 - pow((currentIteration/totalIteration), 4) ) * maxSpeed;

        if (graduateSpeed < maxSpeed/1.5) return max(maxSpeed/1.5, lowest_speed);
        return graduateSpeed;

    }

    float closestMultipleOf90(float number) {
        return round(number / 90.0f) * 90.0f;
    }

    void turnTillTarget(int speed, float target, bool turnLeft, bool (*condition)(float yaw, float target)){
        updateYawReference();
        AppMPU6050getdata.MPU6050_dveGetEulerAngles(&Yaw);
        while (condition(Yaw, target))
        {
            this->updateYawReference();
            // AppMPU6050getdata.MPU6050_dveGetEulerAngles(&Yaw);
            if (turnLeft) this->moveLeft(calculateGradualSpeed(Yaw, target, speed));
            else this->moveRight(calculateGradualSpeed(Yaw, target, speed));
            if (debug)
                Serial.println("Current Yaw: " + String(Yaw) + " | target: " + String(target));
        }

        this->stop();
    }

public:
    void init()
    {
        // Serial.begin(9600);
        AppMotor.DeviceDriverSet_Motor_Init();
        AppMPU6050getdata.MPU6050_dveInit();
        // delay(2000);
        AppMPU6050getdata.MPU6050_calibration();
    }

    void recalibrate(){
        this->stop();
        AppMPU6050getdata.MPU6050_calibration();
        this->updateYawReference();
    }

    void printAngle(){
        AppMPU6050getdata.MPU6050_dveGetEulerAngles(&Yaw);
        Serial.println("Current Angle: " + String(Yaw));
    }

    void moveForward(int speed)
    {
        // this->adjust(10);
        this->stop();
        updateYawReference();
        ApplicationFunctionSet_ConquerorCarMotionControl(
            ConquerorCarMotionControl::Forward,
            speed // 0-255
        );
    }

    void moveForwardForSeconds(int speed, float ms){
        updateYawReference();
        for (int i = 0; i < ms/moveDelayInterval; i ++){
            int speedNow = calculateStraightGraduateSpeed(i+1, ms/5, speed);
            this->moveForward(speedNow);
            if (speedNow < speed) {
                float multiplier = speed/speedNow;
                delay(moveDelayInterval * multiplier);
            }
            else delay(moveDelayInterval);
        }
        this->stop();
    }

    void moveBackwardForSeconds(int speed, float ms){
        updateYawReference();
        for (int i = 0; i < ms/moveDelayInterval; i ++){
            int speedNow = calculateStraightGraduateSpeed(i+1, ms/5, speed);
            this->moveBackward(speedNow);
            if (speedNow < speed) {
                float multiplier = speed/speedNow;
                delay(moveDelayInterval * multiplier);
            }
            else delay(moveDelayInterval);
        }
        this->stop();
    }

    void moveForwardDistance(int speed, float distanceInM){
        this->stop();
        delay(100);
        AppMPU6050getdata.resetDistance();
        float distance = 0;
        delay(100);
        getFreeRAMSpace();
        while (distance < distanceInM){
            distance = AppMPU6050getdata.MPU6050_getDistance('y');
            this->moveForward(speed);
            // delay(4);
            // Serial.println("Distance traveled: " + String(distance) + "m");
        }
        this->stop();
    }

    void moveBackwardDistance(int speed, float distanceInM){
        this->stop();
        delay(100);
        AppMPU6050getdata.resetDistance();
        float distance = 0;
        delay(100);
        this->moveBackward(speed);
        while (distance < distanceInM){
            distance = AppMPU6050getdata.MPU6050_getDistance('y');
            // Serial.println("Distance traveled: " + String(distance) + "m");
        }
        this->stop();
    }

    void moveBackward(int speed)
    {
        this->stop();
        // this->adjust(speed);
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
        this->stop();
        AppMPU6050getdata.MPU6050_dveGetEulerAngles(&Yaw);
        float target = closestMultipleOf90(Yaw - 90);
        turnTillTarget(speed, target, true, [](float Yaw, float target) { return Yaw > target; });
        this->adjust(speed);
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
        this->stop();
        AppMPU6050getdata.MPU6050_dveGetEulerAngles(&Yaw);
        float target = closestMultipleOf90(Yaw + 90);
        turnTillTarget(speed, target, false, [](float Yaw, float target) { return Yaw < target; });
        this->adjust(speed);
    }

    void adjust(int speed){
        
        this->stop();
        AppMPU6050getdata.MPU6050_dveGetEulerAngles(&Yaw);
        float target = closestMultipleOf90(Yaw);
        if (debug) Serial.println("Offset Angle: " + String(Yaw-target));
        // delay(5000);
        if (abs(Yaw - target) <= adjust_threshold) return;

        Serial.println("Adjust Target | Current Yaw: " + String(Yaw) + " | target: " + String(target));

        if (Yaw > target){ // too far right, turn left
            turnTillTarget(speed, target, true, [](float Yaw, float target) { return Yaw > target; });
        }else{ // too far left, turn right
            turnTillTarget(speed, target, false, [](float Yaw, float target) { return Yaw < target; });
        }
        this->stop();
        AppMPU6050getdata.resetYawAtIntervals();
    }

    void stop()
    {
        ApplicationFunctionSet_ConquerorCarMotionControl(
            ConquerorCarMotionControl::stop_it,
            0);
    }
};