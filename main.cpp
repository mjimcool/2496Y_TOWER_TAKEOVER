#include "vex.h"

//#region config_globals
vex::brain Brain;
vex::controller con(vex::controllerType::primary);
vex::motor FrontLeft(vex::PORT1, vex::gearSetting::ratio36_1, true);
vex::motor BackLeft(vex::PORT2, vex::gearSetting::ratio36_1, true);
vex::motor FrontRight(vex::PORT3, vex::gearSetting::ratio36_1, false);
vex::motor BackRight(vex::PORT4, vex::gearSetting::ratio36_1, false);
vex::motor Claw(vex::PORT5, vex::gearSetting::ratio18_1, false);
vex::motor LiftLeft(vex::PORT6, vex::gearSetting::ratio18_1, false);
vex::motor LiftRight(vex::PORT7, vex::gearSetting::ratio18_1, true);
vex::pot liftPot(Brain.ThreeWirePort.A);
vex::encoder leftEncoder(Brain.ThreeWirePort.D);
vex::encoder rightEncoder(Brain.ThreeWirePort.F);
//#endregion config_globals

using namespace vex;

// Creates a competition object that allows access to Competition methods.
vex::competition Competition;


#include "lib.h"
#include "autonomous.h"

void pre_auton() {
    thread lift_pwr( liftPower );
    thread chassis_pwr( chassisPower );

}

void autonomous() {
    chassisPIDEnable = true;
    liftPIDEnable = true;
    sampleAutonName();
}

#include "teleop.h"

int main() {
    // Do not adjust the lines below

    // Set up (but don't start) callbacks for autonomous and driver control periods.
    Competition.autonomous(autonomous);
    Competition.drivercontrol(drivercontrol);

    // Run the pre-autonomous function.
    pre_auton();

    // Robot Mesh Studio runtime continues to run until all threads and
    // competition callbacks are finished.
}