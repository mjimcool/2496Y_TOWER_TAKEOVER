#ifndef AUTONOMOUS_H
#define AUTONOMOUS_H

#include "lib.h"

void stopDriveMotors(){
    FrontLeft.stop();
    FrontRight.stop();
    BackLeft.stop();
    BackRight.stop();
}

#include "movement.h"

void closeClaw(){
    //Claw.startRotateTo(-205, vex::rotationUnits::deg);
    if(!clawCloseState_last){
        clawPIDEnable = true;
    }
    clawCloseState = true;
}
void openClaw(){
    clawPIDEnable = false;
    Claw.startRotateTo(-100+clawOffset, vex::rotationUnits::deg);
}
void openWide(){
    clawPIDEnable = false;
    clawCloseState = false;
    Claw.startRotateTo(-10+clawOffset, vex::rotationUnits::deg);
}
void addStack(){
    requestAddStack = true;
}

void releaseStack(){
    reqReleaseStack = true;
}

void thread_clawControl(){
    while(true){

        //move claw to open or close
        if(!clawOverride && !clawOverride2){
            if(clawCloseState){
                //Claw.startRotateTo(-205, vex::rotationUnits::deg);
                if(!clawCloseState_last){
                    clawPIDEnable = true;
                }
            }
            else{
                openClaw();
            }
        }
        clawCloseState_last = clawCloseState;
        if(con.ButtonRight.pressing()){
            clawOverride2 = true;
            clawCloseState = false;
            openWide();
        }
        sleepMs(10);
    }
}

void thread_addStack(){
    while(true){
        if(requestAddStack){
            clawCloseState = false;
            sleepMs(200);
            if(liftPos > 0){
                liftPos -= 1;
                lift.target = liftValues[liftPos];
            }
            sleepMs(400);
            clawCloseState = true;
            sleepMs(300);
            liftPos += 1;
            lift.target = liftValues[liftPos];
            requestAddStack = false;
        }
        sleepMs(10);
    }
}

void thread_releaseStack(){
    while(true){
        if(reqReleaseStack){
            usingPos = true;
            if(liftPos > 1){
                liftPos -= 1;
            }
            sleepMs(200);
            clawOverride2 = true;
            clawCloseState = false;
            openWide();
            reqReleaseStack = false;
        }
        sleepMs(10);
    }
}

void smallBlue(){
    slewOverrideEnable = true;
    sleepMs(250);
    lift.target = liftValues[0];
    sleepMs(500);
    closeClaw();
    sleepMs(300);
    lift.target = liftValues[2];
    sleepMs(300);
    drive(480, 11000, eBlocking::BLOCKING, 300);
    liftPos = 1;
    lift.target = liftValues[1]-50;
    sleepMs(400);
    addStack();
    sleepMs(1100);
    lift.target = liftValues[2];
    drive(110, 11000, eBlocking::BLOCKING, 500);
    sleepMs(50);
    liftPos = 1;
    lift.target = liftValues[1]-50;
    sleepMs(400);
    addStack();
    sleepMs(900);
    liftPos = 1;
    lift.target = liftValues[1]+50;
    drive(120, 11000, eBlocking::BLOCKING, 500);
    sleepMs(300);
    liftPos = 1;
    lift.target = liftValues[1]-50;
    sleepMs(400);
    addStack();
    sleepMs(800);
    f_chassis.slewRate = 20;
    drive(-450, 4000, eBlocking::BLOCKING);
    f_chassis.slewRate = 150;
    pivot(117, turn_direction::LEFT, 11000, eBlocking::BLOCKING);
    f_chassis.slewRate = 100;
    drive(800, 7000, eBlocking::BLOCKING, 400);
    f_chassis.slewRate = 150;
    liftPos = 0;
    lift.target = liftValues[0];
    sleepMs(500);
    openWide();
    sleepMs(500);
    drive(-720, 11000, eBlocking::BLOCKING);
    
}

void largeBlue(){
    slewOverrideEnable = true;
    sleepMs(250);
    lift.target = liftValues[0];
    sleepMs(500);
    closeClaw();
    sleepMs(300);
    lift.target = liftValues[2];
    sleepMs(300);
    drive(250, 11000, eBlocking::BLOCKING, 300);
    liftPos = 1;
    lift.target = liftValues[1]-50;
    sleepMs(400);
    addStack();
    sleepMs(1000);
    drive(270, 11000, eBlocking::BLOCKING);
    sleepMs(200);
    pivot(90, turn_direction::RIGHT, 11000, eBlocking::NONBLOCKING);
    sleepMs(2500);
    turnPIDEnable = false;
    f_chassis.reset_variables();
    angle.reset_variables();
    stopDriveMotors();
    sleepMs(50);
    lift.target = liftValues[2];
    drive(600, 11000, eBlocking::BLOCKING);
    liftPos = 1;
    lift.target = liftValues[1]-50;
    sleepMs(400);
    addStack();
    sleepMs(1000);
    pivot(27, turn_direction::RIGHT, 11000, eBlocking::BLOCKING);
    drive(785, 8000, eBlocking::BLOCKING, 300);
    liftPos = 0;
    lift.target = liftValues[0];
    sleepMs(500);
    openWide();
    sleepMs(500);
    liftPos = 1;
    lift.target = liftValues[1]-50;
    drive(-720, 11000, eBlocking::BLOCKING);
    
}
void smallRed(){
    slewOverrideEnable = true;
    sleepMs(250);
    lift.target = liftValues[0];
    sleepMs(500);
    closeClaw();
    sleepMs(300);
    lift.target = liftValues[2];
    sleepMs(300);
    drive(480, 11000, eBlocking::BLOCKING, 300);
    liftPos = 1;
    lift.target = liftValues[1]-50;
    sleepMs(400);
    addStack();
    sleepMs(1100);
    lift.target = liftValues[2];
    drive(110, 11000, eBlocking::BLOCKING, 500);
    sleepMs(50);
    liftPos = 1;
    lift.target = liftValues[1]-50;
    sleepMs(400);
    addStack();
    sleepMs(900);
    liftPos = 1;
    lift.target = liftValues[1]+50;
    drive(120, 11000, eBlocking::BLOCKING, 500);
    sleepMs(300);
    liftPos = 1;
    lift.target = liftValues[1]-50;
    sleepMs(400);
    addStack();
    sleepMs(800);
    f_chassis.slewRate = 20;
    drive(-450, 4000, eBlocking::BLOCKING);
    f_chassis.slewRate = 150;
    pivot(117, turn_direction::RIGHT, 11000, eBlocking::BLOCKING);
    f_chassis.slewRate = 100;
    drive(810, 7000, eBlocking::BLOCKING, 400);
    f_chassis.slewRate = 150;
    liftPos = 0;
    lift.target = liftValues[0];
    sleepMs(500);
    openWide();
    sleepMs(500);
    drive(-720, 11000, eBlocking::BLOCKING);
}

void largeRed(){
    slewOverrideEnable = true;
    sleepMs(150);
    lift.target = liftValues[0];
    sleepMs(200);
    closeClaw();
    sleepMs(200);
    lift.target = liftValues[2];
    sleepMs(300);
    drive(250, 11000, eBlocking::BLOCKING, 300);
    liftPos = 1;
    lift.target = liftValues[1]-50;
    sleepMs(400);
    addStack();
    sleepMs(600);
    pivot(97, turn_direction::LEFT, 12000, eBlocking::NONBLOCKING);
    sleepMs(1200);
    turnPIDEnable = false;
    f_chassis.reset_variables();
    angle.reset_variables();
    stopDriveMotors();
    sleepMs(50);
    drive(1220, 8000, eBlocking::BLOCKING);
    liftPos = 0;
    lift.target = liftValues[0];
    sleepMs(300);
    openWide();
    sleepMs(300);
    drive(-1130, 10000, eBlocking::BLOCKING);
    pivot(100, turn_direction::RIGHT, 12000, eBlocking::NONBLOCKING);
    sleepMs(1200);
    turnPIDEnable = false;
    f_chassis.reset_variables();
    angle.reset_variables();
    stopDriveMotors();
    sleepMs(50);
    liftPos = 0;
    lift.target = liftValues[0];
    drive(920, 8000, eBlocking::BLOCKING);
    sleepMs(200);
    closeClaw();
    liftPos = 1;
    lift.target = liftValues[1];
    
}

void testAuton(){
    slewOverrideEnable = true;
    sleepMs(250);
    lift.target = liftValues[0];
    sleepMs(250);
    //drive(2000, 11000, eBlocking::BLOCKING);
    //drive(-2000, 11000, eBlocking::BLOCKING);
    pivot(90, turn_direction::LEFT, 11000, eBlocking::NONBLOCKING);
    sleepMs(10000);
}

#endif