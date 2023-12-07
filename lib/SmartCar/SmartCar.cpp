#include <avr/wdt.h>
#include "DeviceDriverSet_xxx0.h"
#include "ApplicationFunctionSet_xxx0.cpp"


class SmartCar{

    private:
        DeviceDriverSet_Motor AppMotor;
        Application_xxx Application_SmartRobotCarxxx0;
    public:
        SmartCar(){
            // nothing 
        }

        void moveFoward(int speed){
            ApplicationFunctionSet_SmartRobotCarMotionControl(
                SmartRobotCarMotionControl::Forward,
                speed // 0-255
            );
        }

        void moveBackward(int speed){
            ApplicationFunctionSet_SmartRobotCarMotionControl(
                SmartRobotCarMotionControl::Backward,
                speed // 0-255
            );
        }

        void moveLeft(int speed){
            ApplicationFunctionSet_SmartRobotCarMotionControl(
                SmartRobotCarMotionControl::Left,
                speed // 0-255
            );
        }

        void moveRight(int speed){
            ApplicationFunctionSet_SmartRobotCarMotionControl(
                SmartRobotCarMotionControl::Right,
                speed // 0-255
            );
        }

        void stop(){
            ApplicationFunctionSet_SmartRobotCarMotionControl(
                SmartRobotCarMotionControl::stop_it,
                0         
            );
        }
};