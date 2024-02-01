#include "fire.h"
#include "pros/misc.hpp"
#include "pros/rtos.h"
#include <string>

// DRIVE CONTROLS
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
        left_drive[i] = fwd_rev + left_right;
    }

    // apply right speed to right motors
    for (int i = 0; i < right_drive.size(); i++) {
        right_drive[i] = fwd_rev - left_right;
    }
}

void fire::drive::split_arcade_flipped() {
    // get speeds from analog sticks
    int fwd_rev = fire::cont.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
    int left_right = fire::cont.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);

    // apply left speed to left motors
    for (int i = 0; i < left_drive.size(); i++) {
        left_drive[i] = fwd_rev + left_right;
    }

    // apply right speed to right motors
    for (int i = 0; i < right_drive.size(); i++) {
        right_drive[i] = fwd_rev - left_right;
    }
}

// AUTONOMOUS PID CONTROLS

void fire::drive::init_pids() {
    for (int i = 0; i < this->left_drive.size(); i++) {
        this->left_drive[i].tare_position();
    }
    for (int i = 0; i < this->right_drive.size(); i++) {
        this->right_drive[i].tare_position();
    }
    pros::Task(this->drive_pid_task, (void*)this, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "PID Task");
}

/*
        DRIVE PID TASK
*/
void fire::drive::drive_pid_task(void *c) {
    while (pros::competition::is_autonomous()) {
        if (((fire::drive*)c)->current_pid_state == fire::drive::pid_state::Drive) {
            // calculate pid for each left motor
            for (int i = 0; i < ((fire::drive*)c)->left_drive.size(); i++) {
                // error calculation
                float error = ((fire::drive*)c)->left_targets[i] - ((fire::drive*)c)->left_drive[i].get_position();
                float diff = ((fire::drive*)c)->left_prev_errors[i] - error;
                ((fire::drive*)c)->left_prev_errors[i] = error;
                ((fire::drive*)c)->left_total_error[i] += error;

                // p,i,d calculations
                float proportional = ((fire::drive*)c)->drive_Kp * error;
                float integral = ((fire::drive*)c)->drive_Ki * ((fire::drive*)c)->left_total_error[i];
                float derivitive = ((fire::drive*)c)->drive_Kd * diff;

                // calc total power
                float power = proportional + integral + derivitive;

                ((fire::drive*)c)->left_drive[i] = power * ((fire::drive*)c)->speed;
            }
            // calculate pid for each right motor
            for (int i = 0; i < ((fire::drive*)c)->right_drive.size(); i++) {
                // error calculation
                float error = ((fire::drive*)c)->right_targets[i] - ((fire::drive*)c)->right_drive[i].get_position();
                float diff = ((fire::drive*)c)->right_prev_errors[i] - error;
                ((fire::drive*)c)->right_prev_errors[i] = error;
                ((fire::drive*)c)->right_total_error[i] += error;

                // p,i,d calculations
                float proportional = ((fire::drive*)c)->drive_Kp * error;
                float integral = ((fire::drive*)c)->drive_Ki * ((fire::drive*)c)->right_prev_errors[i];
                float derivitive = ((fire::drive*)c)->drive_Kd * diff;

                // calc total power
                float power = proportional + integral + derivitive;

                ((fire::drive*)c)->right_drive[i] = power * ((fire::drive*)c)->speed;
            }
        }
        pros::delay(fire::delay);
    }
}
/*
        DRIVE PID TASK
*/

void fire::drive::stop_pid() {
    this->current_pid_state = this->pid_state::None;
}

void fire::drive::wait_drive() {
    while (true) {
        if (this->current_pid_state == fire::drive::pid_state::Drive) {
            
        }
        pros::delay(fire::delay);
    }
}

void fire::drive::set_exit_conditions(fire::pid_types pid_type, float small_error, float large_error, int small_timeout, int large_timeout, int zero_timeout) {
    if (pid_type == fire::pid_types::Drive) {
        this->drive_small_error = small_error;
        this->drive_large_error = large_error;
        this->drive_small_timeout = small_timeout;
        this->drive_large_timeout = large_timeout;
        this->drive_zero_timeout = zero_timeout;
    } else if (pid_type == fire::pid_types::Turn) {
        this->turn_small_error = small_error;
        this->turn_large_error = large_error;
        this->turn_small_timeout = small_timeout;
        this->turn_large_timeout = large_timeout;
        this->turn_zero_timeout = zero_timeout;
    } else if (pid_type == fire::pid_types::Swing) {
        this->swing_small_error = small_error;
        this->swing_large_error = large_error;
        this->swing_small_timeout = small_timeout;
        this->swing_large_timeout = large_timeout;
        this->swing_zero_timeout = zero_timeout;
    }
}

void fire::drive::set_drive_pid(float distance, int speed) {
    // set left motor targets
    for (int i = 0; i < this->left_drive.size(); i++) {
        float start = left_drive[i].get_position();
        float target = (distance/this->diameter) * (200/this->gearRatio);
        if (left_drive[i].is_reversed()) {
            target = start-target;
        } else {
            target = start+target;
        }
        this->left_targets[i] = target;
    }

    // set right motor targets
    for (int i = 0; i < this->right_drive.size(); i++) {
        float start = right_drive[i].get_position();
        float target = distance/this->diameter;
        if (right_drive[i].is_reversed()) {
            target = start-target;
        } else {
            target = start+target;
        }
        this->right_targets[i] = target;
    }
}
