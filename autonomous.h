#ifndef AUTONOMOUS_H
#define AUTONOMOUS_H

#include "lib.h"

void drive(int target, int maxVoltage, eBlocking ifBlocking){
    odoReset();
    chassisPIDEnable    = true;
    leftChassis.target  = target;
    rightChassis.target = target;
    leftChassis.max_mV  = maxVoltage;
    rightChassis.max_mV = maxVoltage;
    
    if(ifBlocking == eBlocking::BLOCKING){
        while(fabs(leftChassis.error) < 10 && fabs(rightChassis.error) < 10){sleepMs(50);}
        sleepMs(100);
        leftChassis.reset_variables();
        rightChassis.reset_variables();
        chassisPIDEnable = false;
    }
}

void pivot(int degrees, int direction, int maxVoltage, eBlocking ifBlocking){
    odoReset();
    chassisPIDEnable    = true;
    leftChassis.target  = direction*degrees*kLE;
    rightChassis.target = -direction*degrees*kRE;
    leftChassis.max_mV  = maxVoltage;
    rightChassis.max_mV = maxVoltage;

    if(ifBlocking == eBlocking::BLOCKING){
        while(fabs(leftChassis.error) < 10 && fabs(rightChassis.error) < 10){sleepMs(50);}
        sleepMs(100);
        leftChassis.reset_variables();
        rightChassis.reset_variables();
        chassisPIDEnable = false;
    }
}

void closeClaw(){
    Claw.startRotateTo(-170, vex::rotationUnits::deg);
}
void openClaw(){
    Claw.startRotateTo(-90, vex::rotationUnits::deg);
}

void sampleAutonName(){
    closeClaw();
    sleepMs(300);
    lift.target = liftValues[1];
    drive(1000, 12000, eBlocking::BLOCKING);
    lift.target = liftValues[0];
    openClaw();
    sleepMs(500);
    closeClaw();
    drive(-600, 12000, BLOCKING);
    pivot(135, turn_direction::RIGHT, 12000, eBlocking::BLOCKING);

}

#endif