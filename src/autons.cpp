#include "autons.h"

const int DRIVE_SPEED = 110;
const int TURN_SPEED = 80;
const int SWING_SPEED = 80;

void set_exit_conditions() {
    chassis.set_pid(
        fire::pid_types::Drive,
        2.0,
        0.0,
        10.0
    );
    chassis.set_pid(
        fire::pid_types::Turn,
        0.05,
        0.0,
        0.5
    );

    chassis.set_exit_conditions(
        fire::pid_types::Drive,
        3,
        7,
        500,
        5000,
        5000
    );
    chassis.set_exit_conditions(
        fire::pid_types::Turn,
        3,
        7,
        50,
        100,
        500
    );
    chassis.set_exit_conditions(
        fire::pid_types::Swing,
        0.5,
        3,
        150,
        350,
        500
    );
}

void test_auton() {
    fire::lcd::print("Auton Started");
    
    fire::lcd::print("Driving Forward");
    chassis.set_drive_pid(24, DRIVE_SPEED);
    chassis.wait_until(12);
    // do something after 12 inches
    chassis.wait_drive();

    fire::lcd::print("Driving Backwards");
    chassis.set_drive_pid(-24, DRIVE_SPEED);
    chassis.wait_drive();
    
    fire::lcd::print("Turning Right");
    chassis.set_turn_pid(90, TURN_SPEED);
    chassis.wait_drive();

    fire::lcd::print("Turning Left");
    chassis.set_turn_pid(-90, TURN_SPEED);
    chassis.wait_drive();

    fire::lcd::print("Straightning");
    chassis.set_turn_pid(0, TURN_SPEED);
    chassis.wait_drive();



    fire::lcd::print("Auton Finished");
    fire::cont.rumble(".");
}