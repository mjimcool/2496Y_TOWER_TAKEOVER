#ifndef TELEOP_H
#define TELEOP_H

#include "lib.h"

void drivercontrol() {

    //initialize PID with constants 1, 0, 0
    lift.pidInit(1, 0, 0);

    //set claw to move at max velocity
    Claw.setVelocity(100, vex::velocityUnits::pct);

    //creating a thread for lift PID control
    thread liftypowerthing( liftPower );
    while (true) {

        //standard voltage controlled tank-style drive
        tankControl();

        //hold to turn on shift
        btnShift = con.ButtonR2.pressing() ? true : false;

        //button toggle for claw control
        if(con.ButtonR1.pressing() && !stateR1){
            clawCloseState = !clawCloseState;
        }
        stateR1 = con.ButtonR1.pressing();

        //move claw to open or close
        if(clawCloseState){
            Claw.startRotateTo(-170, vex::rotationUnits::deg);
        }
        else{
            Claw.startRotateTo(-90, vex::rotationUnits::deg);
        }
        
        /*  if(con.ButtonL1.pressing()){
            lift.target++;
        }
        else if(con.ButtonL2.pressing()){
            lift.target--;
        }
        */
 

        //left triggers add and subtract to lift pos, if shift, add and subtract 3
        if(con.ButtonL1.pressing() && liftPos < 10 && !btnShift && !stateL1){
            liftPos++;
        }
        else if(con.ButtonL1.pressing() && liftPos < 10 && btnShift && !stateL1){
            if(liftPos > 7){
                liftPos = 10;
            }
            else{
                liftPos += 3;           
            }
        }
        else if(con.ButtonL2.pressing() && liftPos > 0 && !btnShift && !stateL2){
            liftPos--;
        }
        else if(con.ButtonL2.pressing() && liftPos > 0 && btnShift && !stateL2){
            if(liftPos < 3){
                liftPos = 0;
            }
            else{
                liftPos -= 3;
            }
        }

        stateL1 = con.ButtonL1.pressing();
        stateL2 = con.ButtonL2.pressing();

        lift.target = liftValues[liftPos];

        //tipping safety control
        if(liftPos > 5 && liftPos <= 7){ 
            max_mV_driver = 9000;
        }
        else if(liftPos > 7){
            max_mV_driver = 7000;
        }
        else{
            max_mV_driver = 12000;
        }

       sleepMs(10);
    }
}

#endif