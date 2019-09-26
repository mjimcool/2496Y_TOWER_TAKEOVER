#ifndef MOVEMENT_H
#define MOVEMENT_H

#include "lib.h"

void drive(int target, int maxVoltage, eBlocking ifBlocking){
    f_chassis.reset_variables();
    angle.reset_variables();
    odoReset();
    f_chassis.pidInit(28, 4, 253, 2000);//25 1.25, 400
    chassisPIDEnable    = true;
    f_chassis.target  = target;
    f_chassis.max_mV  = maxVoltage;
    sleepMs(25);
    if(ifBlocking == eBlocking::BLOCKING){
        while(fabs(f_chassis.error) > 12){sleepMs(10);}
        chassisPIDEnable = false;
        f_chassis.reset_variables();
        angle.reset_variables();
        stopDriveMotors();
        sleepMs(50);
    }
}

void drive(int target, int maxVoltage, eBlocking ifBlocking, int timer){
    f_chassis.reset_variables();
    angle.reset_variables();
    odoReset();
    f_chassis.pidInit(28, 4, 253, 2000);
    chassisPIDEnable    = true;
    f_chassis.target  = target;
    f_chassis.max_mV  = maxVoltage;
    sleepMs(25);
    int i = 0;
    if(ifBlocking == eBlocking::BLOCKING){
        while(fabs(f_chassis.error) > 12){
            i++;
            sleepMs(10);
            if(i >= timer){
                break;
            }
        }
        chassisPIDEnable = false;
        f_chassis.reset_variables();
        angle.reset_variables();
        stopDriveMotors();
        sleepMs(50);
    }
}

void pivot(int degrees, int direction, int maxVoltage, eBlocking ifBlocking){
    f_chassis.reset_variables();
    angle.reset_variables();
    odoReset();
    f_chassis.pidInit(35, .5, 50, 8000);
    turnPIDEnable = true;
    float turnConst = direction > 0 ? 2.35 : 2.35;
    f_chassis.target  = direction*degrees*turnConst;
    f_chassis.max_mV  = maxVoltage;
    sleepMs(25);
    if(ifBlocking == eBlocking::BLOCKING){
        while(fabs(f_chassis.error) > 3){sleepMs(10);}
        sleepMs(50);
        turnPIDEnable = false;
        f_chassis.reset_variables();
        angle.reset_variables();
        stopDriveMotors();
        sleepMs(25);
    }
}
void pivot(int degrees, int direction, int maxVoltage, eBlocking ifBlocking, int timer){
    f_chassis.reset_variables();
    angle.reset_variables();
    odoReset();
    f_chassis.pidInit(35, 0.5, 50, 5000);
    turnPIDEnable = true;
    float turnConst = direction > 0 ? 2.35 : 2.35;
    f_chassis.target  = direction*degrees*turnConst;
    f_chassis.max_mV  = maxVoltage;
    sleepMs(25);
    int i = 0;
    if(ifBlocking == eBlocking::BLOCKING){
        while(fabs(f_chassis.error) > 10){
            i++;
            sleepMs(10);
            if(i >= timer){
                break;
            }
        }
        sleepMs(50);
        turnPIDEnable = false;
        f_chassis.reset_variables();
        angle.reset_variables();
        stopDriveMotors();
        sleepMs(25);
    }
}
#endif