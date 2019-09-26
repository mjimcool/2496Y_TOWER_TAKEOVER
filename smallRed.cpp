#include <cstdlib>
#include <cmath>
#include "vex.h"

//#region config_globals
vex::brain Brain;
vex::controller con(vex::controllerType::primary);
vex::motor FrontLeft(vex::PORT1, vex::gearSetting::ratio36_1, true);
vex::motor BackLeft(vex::PORT2, vex::gearSetting::ratio36_1, true);
vex::motor FrontRight(vex::PORT3, vex::gearSetting::ratio36_1, false);
vex::motor BackRight(vex::PORT4, vex::gearSetting::ratio36_1, false);
vex::motor LiftLeft(vex::PORT6, vex::gearSetting::ratio36_1, false);
vex::motor LiftRight(vex::PORT7, vex::gearSetting::ratio36_1, true);
vex::motor Claw(vex::PORT8, vex::gearSetting::ratio36_1, false);
vex::pot liftPot(Brain.ThreeWirePort.A);
vex::encoder leftEncoder(Brain.ThreeWirePort.C);
vex::encoder rightEncoder(Brain.ThreeWirePort.E);
//#endregion config_globals

using namespace vex;

// Creates a competition object that allows access to Competition methods.
vex::competition Competition;


#include "lib.h"
#include "autonomous.h"

void pre_auton() {
    thread lift_pwr( thread_liftPower );
    thread chassis_pwr( thread_chassisPower );
    thread ODODO( thread_odoTracking );
    thread DEBUG( thread_debugStream );
    thread stadd( thread_addStack );
    thread relea( thread_releaseStack );
    thread claww( thread_clawPower );
    thread claw2( thread_clawControl );

    //set claw to move at half velocity
    Claw.setVelocity(50, vex::velocityUnits::pct);

    odoReset();

    liftPos = 0;
    lift.target = 900;
    o_claw.target = 2.3; //target 2.3 newton-meters - stall torque 2.1 (not necessarily overheating motors)

    o_claw.i_bound = 0.2;
    f_chassis.i_bound = 100;

    f_chassis.slewRate = 150;
    lift.pidInit(28, 0.5, 0, 8000);
    angle.pidInit(5, 0, 0, 1000);
    o_claw.pidInit(600, 80, 0, 50);
}

void autonomous() {
    liftPIDEnable = true;
    smallRed();
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