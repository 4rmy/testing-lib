#include "autons.h"

const int DRIVE_SPEED = 110;
const int TURN_SPEED = 80;
const int SWING_SPEED = 80;

void set_exit_conditions() {
    chassis.set_pid(fire::pid_types::Drive, 0.75, 0.01, 0.0);
    chassis.set_pid(fire::pid_types::Turn, 0.01, 0.0001, 0.0);

    chassis.set_exit_conditions(fire::pid_types::Drive, 1, 3, 150, 350, 500);
    chassis.set_exit_conditions(fire::pid_types::Turn, 0.5, 3, 150, 350, 500);
    chassis.set_exit_conditions(fire::pid_types::Swing, 0.5, 3, 150, 350, 500);
}

void test_auton() {
    fire::lcd::print("Auton Started");

    fire::lcd::print("Driving Forward");
    chassis.set_drive_pid(12, DRIVE_SPEED);
    chassis.wait_drive();

    fire::lcd::print("Driving Backwards");
    chassis.set_drive_pid(-12, DRIVE_SPEED);
    chassis.wait_drive();

    fire::lcd::print("Turning Right");
    chassis.set_turn_pid(90, TURN_SPEED);
    chassis.wait_drive();

    fire::lcd::print("Turning Left");
    chassis.set_turn_pid(-90, TURN_SPEED);
    chassis.wait_drive();

    fire::lcd::print("Turning Straight");
    chassis.set_turn_pid(0, TURN_SPEED);
    chassis.wait_drive();

    fire::lcd::print("Auton Finished");
    fire::cont.rumble(".");
}