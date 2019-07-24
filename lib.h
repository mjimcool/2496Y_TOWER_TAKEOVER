#ifndef LIB_H
#define LIB_H

//variables for axis output
int  axis2             = 0;
int  axis3             = 0;

int  max_mV_driver     = 12000;

//shift variable
bool btnShift          = false;

//variable to store lift position
int  liftPos           = 0;

//button states used for rising-edge trigger
bool stateL1           = false;
bool stateL2           = false;
bool stateR1           = false;

//claw state
bool clawCloseState    = false;

//pid toggle state
bool chassisPIDEnable  = false;
bool liftPIDEnable     = false;

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
bool chassisPIDEnable = false;
//turning constants
const float kLE        = 1;
const float kRE        = 1;

//encoder values from odometry wheels
int odoLeft            = 0;
int odoRight           = 0;

const int liftValues[] = {600, 800, 1000, 1200, 1400, 1600, 1800, 2000, 2200, 2400};

float sgn(float input){
  if(input>1){
    return 1;
  }
  else if(input<1){
    return -1;
  }
  else{
    return 0;
  }
}

float fabs(float input){
    if(input<0){
        return -input;
    }
    else{
        return input;
    }
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

class PidControl{
    public:
    int   currentPosition = 0;
    float error           = 0;
    float integral        = 0;
    float derivative      = 0;
    float last_error      = 0;
    
    float kP              = 0;
    float kI              = 0;
    float kD              = 0;

    float target          = 0;
    int   max_mV          = 12000;

    int   output          = 0;

    void pidInit(float p, float i, float d){
        kP = p;
        kI = i;
        kD = d;
    }

    void moveTo(){
        error = target - currentPosition;
        if(fabs(error) < 50){
            if(fabs(integral) < 4000)
                integral += error;
            else{
                integral = sgn(integral)*4000;
            }   
        }
        derivative = error - last_error;
        last_error = currentPosition;

        output = kP*error + kI*integral + kD*derivative;

        output = fabs(output) > max_mV ? sgn(output)*max_mV : output;

        sleepMs(10);
    }

    void reset_variables(){
        currentPosition = 0;
        error           = 0;
        integral        = 0;
        derivative      = 0;
        last_error      = 0;
        target          = 0;
    }
};
PidControl lift;
PidControl leftChassis;
PidControl rightChassis;

void odoTracking(){
    odoLeft = leftEncoder.rotation(vex::rotationUnits::deg);
    odoRight = rightEncoder.rotation(vex::rotationUnits::deg);
    sleepMs(10);
}

void odoReset(){
    leftEncoder.resetRotation();
    rightEncoder.resetRotation();
}

void liftPower(){
    while(true){
        if(liftPIDEnable){
            lift.currentPosition = liftPot.value(analogUnits::range12bit);
            lift.moveTo();

            LiftLeft.spin(vex::directionType::fwd, lift.output, vex::voltageUnits::mV);
            LiftRight.spin(vex::directionType::fwd, lift.output, vex::voltageUnits::mV);
            sleepMs(10);
        }
    }
}

void chassisPower(){
    while(true){
        if(chassisPIDEnable){
            leftChassis.currentPosition = odoLeft;
            rightChassis.currentPosition = odoRight;
            leftChassis.moveTo();
            rightChassis.moveTo();

            FrontLeft.spin(vex::directionType::fwd, leftChassis.output, vex::voltageUnits::mV);
            BackLeft.spin(vex::directionType::fwd, leftChassis.output, vex::voltageUnits::mV);
            FrontRight.spin(vex::directionType::fwd, rightChassis.output, vex::voltageUnits::mV);
            BackRight.spin(vex::directionType::fwd, rightChassis.output, vex::voltageUnits::mV);
            sleepMs(10);
        }
    }
}

void debugStream(){
    while(true){
        Brain.Screen.clearScreen();
        Brain.Screen.setCursor(1,1);
        Brain.Screen.print(lift.currentPosition);
        Brain.Screen.newLine();
        Brain.Screen.print(lift.target);
        Brain.Screen.newLine();
        Brain.Screen.print(lift.output);

        sleepMs(200);
        
    }
}

#endif