#ifndef LIB_H
#define LIB_H

#include "pidcontrol.h"

//variables for axis output
int  axis2             = 0;
int  axis3             = 0;

int  max_mV_driver     = 12000;

//shift variable
bool btnShift          = false;

//variable to store lift position
int  liftPos           = 0;
int  towerPos          = 0;
bool usingPos          = false;
bool usingPos_t        = false;

//button states used for rising-edge trigger
bool stateL1           = false;
bool stateL2           = false;
bool stateR1           = false;
bool stateB            = false;

//claw state
bool clawCloseState    = false;
bool clawCloseState_last = false;
bool clawOverride      = false;
bool clawOverride2     = false;

//pid toggle state
bool chassisPIDEnable  = false;
bool liftPIDEnable     = false;
bool clawPIDEnable     = false;
bool turnPIDEnable     = false;
bool slewOverrideEnable = false;

bool requestAddStack   = false;
bool reqReleaseStack   = false;

int clawOffset = 0;

int lift_last_target   = 0;

//definitions of the enums used to increase code readability
namespace turn_direction{
    enum eDirection{
        LEFT = -1,
        RIGHT = 1
    };
}
enum class eBlocking{
    BLOCKING,
    NONBLOCKING
};

//turning constants
const float kLE        = 2.61;

//encoder values from odometry wheels
float odoLeft            = 0;
float odoRight           = 0;

const int liftValues[] = {980, 1340, 1530, 1780, 1990, 2270, 2490, 2830};
const int towerValues[] = {980, 1930, 2270, 2890};

template <typename T> int sgn(T val){
    return (T(0) < val) - (val < T(0));
}

void tankControl()
{
    axis2 = con.Axis2.position(percentUnits::pct)*120;
    axis3 = con.Axis3.position(percentUnits::pct)*120;

    axis2 = fabs(axis2) > max_mV_driver ? sgn(axis2)*max_mV_driver : axis2;
    axis3 = fabs(axis2) > max_mV_driver ? sgn(axis3)*max_mV_driver : axis3;
    
    FrontLeft.spin(vex::directionType::fwd, axis3, vex::voltageUnits::mV);
    BackLeft.spin(vex::directionType::fwd, axis3, vex::voltageUnits::mV);
    FrontRight.spin(vex::directionType::fwd, axis2, vex::voltageUnits::mV);
    BackRight.spin(vex::directionType::fwd, axis2, vex::voltageUnits::mV);
}

void thread_odoTracking(){
    while(true){
        odoLeft = 0.975*leftEncoder.value();
        odoRight = rightEncoder.value();
        sleepMs(10);
    }
}

void odoReset(){
    leftEncoder.resetRotation();
    rightEncoder.resetRotation();
    odoLeft = 0;
    odoRight = 0;
}

void thread_liftPower(){
    while(true){
        if(liftPIDEnable){
            lift.currentPosition = liftPot.value(analogUnits::range12bit);
            lift.moveTo();

            LiftLeft.spin(vex::directionType::fwd, lift.output, vex::voltageUnits::mV);
            LiftRight.spin(vex::directionType::fwd, lift.output, vex::voltageUnits::mV);
        }
        sleepMs(10);
    }
}

void thread_chassisPower(){
    while(true){
        if(chassisPIDEnable){
            f_chassis.currentPosition = odoLeft;
            angle.target = odoLeft;
            angle.currentPosition = odoRight;
            f_chassis.moveTo();
            if(slewOverrideEnable){
                f_chassis.slewOverride();
            }
            angle.moveTo();

            FrontLeft.spin(vex::directionType::fwd, f_chassis.output-angle.output, vex::voltageUnits::mV);
            BackLeft.spin(vex::directionType::fwd, f_chassis.output-angle.output, vex::voltageUnits::mV);
            FrontRight.spin(vex::directionType::fwd, f_chassis.output+angle.output, vex::voltageUnits::mV);
            BackRight.spin(vex::directionType::fwd, f_chassis.output+angle.output, vex::voltageUnits::mV);
        }
        else if(turnPIDEnable){
            f_chassis.currentPosition = (odoLeft-odoRight)/2;//average the two
            f_chassis.moveTo();
            if(slewOverrideEnable){
                f_chassis.slewOverride();
            }

            FrontLeft.spin(vex::directionType::fwd, f_chassis.output, vex::voltageUnits::mV);
            BackLeft.spin(vex::directionType::fwd, f_chassis.output, vex::voltageUnits::mV);
            FrontRight.spin(vex::directionType::rev, f_chassis.output, vex::voltageUnits::mV);
            BackRight.spin(vex::directionType::rev, f_chassis.output, vex::voltageUnits::mV);
        }
        sleepMs(10);
    }
}

//experimental claw torque PID
void thread_clawPower(){
    while(true){
        if(clawPIDEnable)
        {
            if(Claw.rotation(vex::rotationUnits::deg) < -260+clawOffset){
                 clawCloseState = false;
            }

            o_claw.currentPosition = Claw.torque(vex::torqueUnits::Nm);
            o_claw.moveTo();

            Claw.spin(vex::directionType::rev, 8000+o_claw.output, vex::voltageUnits::mV);
        }
        sleepMs(10);
    }
}


void thread_debugStream(){
    while(true){
        Brain.Screen.clearScreen();
        Brain.Screen.setCursor(1,1);

        Brain.Screen.print(liftPot.value(analogUnits::range12bit));
        Brain.Screen.newLine();
        Brain.Screen.print(odoLeft);
        Brain.Screen.newLine();
        Brain.Screen.print(odoRight);
        Brain.Screen.newLine();
        Brain.Screen.print(f_chassis.output);
        Brain.Screen.newLine();
        con.Screen.print(f_chassis.error);
        printf("OUTPUT: %.4f \n",(float)o_claw.output);

        sleepMs(100);
        
    }
}

#endif