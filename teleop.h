#ifndef TELEOP_H
#define TELEOP_H

#include "lib.h"

void drivercontrol() {

    liftPIDEnable = true;
    chassisPIDEnable = false;

    //creating a thread for lift PID control
    while (true) {

        //standard voltage controlled tank-style drive
        tankControl();

        //hold to turn on shift
        btnShift = con.ButtonR2.pressing() ? true : false;
        
        //button toggle for claw control
        if(con.ButtonR1.pressing() && !stateR1){
            clawCloseState = !clawCloseState;
            clawOverride2 = false;
        }
        stateR1 = con.ButtonR1.pressing();
        
        if(con.ButtonUp.pressing()){
            usingPos = false;
            usingPos_t = false;
            lift.target+=15;
        }
        else if(con.ButtonLeft.pressing()){
            usingPos = false;
            usingPos_t = false;
            lift.target-=15;
        }
        

        //press button to auto-reverse-stack
        if(con.ButtonY.pressing()){
            addStack();
        }
        if(con.ButtonDown.pressing()){
            releaseStack();
        }

        //left triggers add and subtract to lift pos, if shift, add and subtract 3
        if(con.ButtonL1.pressing() && liftPos < 7 && !btnShift && !stateL1){
            liftPos++;
            usingPos = true;
        }
        else if(con.ButtonL1.pressing() && liftPos < 7 && btnShift && !stateL1){
            usingPos = true;
            if(liftPos > 4){
                liftPos = 7;
            }
            else{
                liftPos += 3;           
            }
        }
        else if(con.ButtonL2.pressing() && liftPos > 0 && !btnShift && !stateL2){
            liftPos--;
            usingPos = true;
        }
        else if(con.ButtonL2.pressing() && liftPos > 0 && btnShift && !stateL2){
            usingPos = true;
            if(liftPos < 3){
                liftPos = 0;
            }
            else{
                liftPos -= 3;
            }
        }

        if(con.ButtonB.pressing() && !stateB){
            usingPos_t = true;
            usingPos = false;
            if(towerPos == 0){
                towerPos = 1;
                liftPos = 3;
            }
            else if(towerPos == 1){
                towerPos = 2;
                liftPos = 5;
            }
            else if(towerPos == 2){
                towerPos = 3;
                liftPos = 7;
            }
            else if(towerPos == 3){
                towerPos = 0;
                liftPos = 0;
            } 
        }

        stateL1 = con.ButtonL1.pressing();
        stateL2 = con.ButtonL2.pressing();
        stateB = con.ButtonB.pressing();
        
        if(!requestAddStack && usingPos){
            usingPos_t = false;
            lift.target = liftValues[liftPos];
        }
        else if(!requestAddStack && usingPos_t){
            lift.target = towerValues[towerPos];
        }

        if(lift.target - lift_last_target != 0){
            lift.reset_integral();
        }

        lift_last_target = lift.target;

        //tipping safety control
        if(liftPos > 3 && liftPos <= 4){ 
            max_mV_driver = 8000;
        }
        else if(liftPos > 5){
            max_mV_driver = 6000;
        }
        else{
            max_mV_driver = 12000;
        }
        if(con.ButtonX.pressing()){
            clawOffset++;
        }
        if(con.ButtonA.pressing()){
            clawOffset--;
        }
        
       sleepMs(10);
    }
}

#endif