#include "autons.h"

const int DRIVE_SPEED = 110;
const int TURN_SPEED = 80;
const int SWING_SPEED = 80;

void set_exit_conditions() {
    chassis.set_pid(fire::pid_types::Drive, 0.50, 0.0, 0.0);
    chassis.set_exit_conditions(fire::pid_types::Drive, 2, 10, 80, 300, 500);
    chassis.set_exit_conditions(fire::pid_types::Turn, 3, 7, 100, 350, 500);
    chassis.set_exit_conditions(fire::pid_types::Swing, 3, 7, 100, 300, 500);
}

void test_auton() {
    fire::lcd::print("Auton Started");
    chassis.set_drive_pid(12, DRIVE_SPEED);
    chassis.wait_drive();
    fire::lcd::print("Auton Finished");
}