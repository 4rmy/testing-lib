#include "fire.h"

void fire::drive::tank_control() {
    // get speeds from analog sticks
    int left_speed = fire::cont.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    int right_speed = fire::cont.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);

    // apply left speed to left motors
    for (int i = 0; i < left_drive.size(); i++) {
        left_drive[i] = left_speed;
    }

    // apply right speed to right motors
    for (int i = 0; i < right_drive.size(); i++) {
        right_drive[i] = right_speed;
    }
}

void fire::drive::split_arcade() {
    // get speeds from analog sticks
    int fwd_rev = fire::cont.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    int left_right = fire::cont.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

    // apply left speed to left motors
    for (int i = 0; i < left_drive.size(); i++) {
        left_drive[i] = fwd_rev - left_right;
    }

    // apply right speed to right motors
    for (int i = 0; i < right_drive.size(); i++) {
        right_drive[i] = fwd_rev + left_right;
    }
}

void fire::drive::split_arcade_flipped() {
    // get speeds from analog sticks
    int fwd_rev = fire::cont.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
    int left_right = fire::cont.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);

    // apply left speed to left motors
    for (int i = 0; i < left_drive.size(); i++) {
        left_drive[i] = fwd_rev - left_right;
    }

    // apply right speed to right motors
    for (int i = 0; i < right_drive.size(); i++) {
        right_drive[i] = fwd_rev + left_right;
    }
}