#include "fire.h"
#include "pros/abstract_motor.hpp"
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

void fire::drive::set_break_mode(pros::MotorBrake mode) {
    for (int i = 0; i < this->left_drive.size(); i++) {
        this->left_drive[i].set_brake_mode(mode);
    }
    for (int i = 0; i < this->right_drive.size(); i++) {
        this->right_drive[i].set_brake_mode(mode);
    }
}

// AUTONOMOUS PID CONTROLS

void fire::drive::init_pids() {
    pros::Task(this->drive_pid_task, (void*)this, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "PID Task");
    this->imu.tare();
}

/*
        DRIVE PID TASK
*/
void fire::drive::drive_pid_task(void *c) {
    while (true) {
        if (((fire::drive*)c)->current_pid_state == fire::drive::pid_state::Drive) {
            // calculate pid for each left motor
            for (int i = 0; i < ((fire::drive*)c)->left_drive.size(); i++) {
                // error calculation
                float error = ((fire::drive*)c)->left_targets[i] - ((fire::drive*)c)->left_drive[i].get_position();
                float diff = error - ((fire::drive*)c)->left_prev_errors[i];
                ((fire::drive*)c)->left_prev_errors[i] = error;
                ((fire::drive*)c)->left_total_error[i] += error;

                // p,i,d calculations
                float proportional = ((fire::drive*)c)->drive_Kp * error;
                float integral = ((fire::drive*)c)->drive_Ki * ((fire::drive*)c)->left_total_error[i];
                float derivitive = ((fire::drive*)c)->drive_Kd * diff;

                // calc total power
                float power = proportional + integral + derivitive;

                float volts = power * ((fire::drive*)c)->speed;
                if (volts < -((fire::drive*)c)->speed) {
                    volts = -127;
                } else if (volts > ((fire::drive*)c)->speed) {
                    volts = 127;
                }

                ((fire::drive*)c)->left_drive[i] = volts;
            }
            // calculate pid for each right motor
            for (int i = 0; i < ((fire::drive*)c)->right_drive.size(); i++) {
                // error calculation
                float error = ((fire::drive*)c)->right_targets[i] - ((fire::drive*)c)->right_drive[i].get_position();
                float diff = error - ((fire::drive*)c)->right_prev_errors[i];
                ((fire::drive*)c)->right_prev_errors[i] = error;
                ((fire::drive*)c)->right_total_error[i] += error;

                // p,i,d calculations
                float proportional = ((fire::drive*)c)->drive_Kp * error;
                float integral = ((fire::drive*)c)->drive_Ki * ((fire::drive*)c)->right_total_error[i];
                float derivitive = ((fire::drive*)c)->drive_Kd * diff;

                // calc total power
                float power = proportional + integral + derivitive;

                float volts = power * ((fire::drive*)c)->speed;
                if (volts < -((fire::drive*)c)->speed) {
                    volts = -127;
                } else if (volts > ((fire::drive*)c)->speed) {
                    volts = 127;
                }

                ((fire::drive*)c)->right_drive[i] = volts;
            }
        } else if (((fire::drive*)c)->current_pid_state == fire::drive::pid_state::Turn) {
            float error = ((fire::drive*)c)->deg_target - ((fire::drive*)c)->imu.get_rotation();
            float diff = error - ((fire::drive*)c)->deg_prev_error;
            ((fire::drive*)c)->deg_prev_error = error;
            ((fire::drive*)c)->deg_total_error += error;

            float proportional = ((fire::drive*)c)->turn_Kp * error;
            float integral = ((fire::drive*)c)->turn_Ki * ((fire::drive*)c)->deg_total_error;
            float derivitive = ((fire::drive*)c)->turn_Kd * diff;

            float power = proportional + integral + derivitive;

            float volts = power * ((fire::drive*)c)->speed;
            if (volts < -((fire::drive*)c)->speed) {
                volts = -127;
            } else if (volts > ((fire::drive*)c)->speed) {
                volts = 127;
            }

            for (int i = 0; i < ((fire::drive*)c)->left_drive.size(); i++) {
                ((fire::drive*)c)->left_drive[i] = volts;
            }
            for (int i = 0; i < ((fire::drive*)c)->right_drive.size(); i++) {
                ((fire::drive*)c)->right_drive[i] = -volts;
            }
        } else if (((fire::drive*)c)->current_pid_state == fire::drive::pid_state::None) {
            for (int i = 0; i < ((fire::drive*)c)->left_drive.size(); i++) {
                ((fire::drive*)c)->left_drive[i] = 0;
            }
            for (int i = 0; i < ((fire::drive*)c)->right_drive.size(); i++) {
                ((fire::drive*)c)->right_drive[i] = 0;
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
    int zero_start = 0;
    int large_start = 0;
    int small_start = 0;
    bool zero_time = false;
    bool large_time = false;
    bool small_time = false;

    while (true) {
        if (this->current_pid_state == fire::drive::pid_state::Drive) {
            if (!zero_time) {
                zero_start = pros::millis();
            } else if (pros::millis() > zero_start + this->drive_zero_timeout) {
                break;
            }

            if (!large_time) {
                large_start = pros::millis();
            } else if (pros::millis() > large_start + this->drive_large_timeout) {
                break;
            }

            if (!small_time) {
                small_start = pros::millis();
            } else if (pros::millis() > small_start + this->drive_small_timeout) {
                break;
            }

            small_time = true;
            large_time = true;
            zero_time = true;
            for (int i = 0; i < this->left_drive.size(); i++) {
                float distance = std::abs((this->left_prev_errors[i] * (200/this->gearRatio)) / this->diameter);

                if (!(distance < this->drive_small_error)) {
                    small_time = false;
                }
                if (!(distance < this->drive_large_error)) {
                    large_time = false;
                }
            }
            for (int i = 0; i < this->right_drive.size(); i++) {
                int distance = std::abs((this->right_prev_errors[i] * (200/this->gearRatio)) / this->diameter);
                if (!(distance < this->drive_small_error)) {
                    small_time = false;
                }
                if (!(distance < this->drive_large_error)) {
                    large_time = false;
                }
            }
            float avg_speed = (this->imu.get_accel().x + this->imu.get_accel().y + this->imu.get_accel().z)/3;
            if (!(std::abs(avg_speed) < 0.4)) {
                zero_time = false;
            }
        } else if (this->current_pid_state == fire::drive::pid_state::Turn) {
            if (!zero_time) {
                zero_start = pros::millis();
            } else if (pros::millis() > zero_start + this->turn_zero_timeout) {
                break;
            }
            
            if (!large_time) {
                large_start = pros::millis();
            } else if (pros::millis() > large_start + this->turn_large_timeout) {
                break;
            }

            if (!small_time) {
                small_start = pros::millis();
            } else if (pros::millis() > small_start + this->turn_small_timeout) {
                break;
            }
        
            small_time = true;
            large_time = true;
            zero_time = true;
            float deg = std::abs(this->deg_target - this->imu.get_rotation());
            if (!(deg < this->turn_small_error)) {
                small_time = false;
            }
            if (!(deg < this->turn_large_error)) {
                large_time = false;
            }
            if (!(std::abs(this->deg_prev_error) < 0.5)) {
                zero_time = false;
            }
        }

        pros::delay(fire::delay);
    }
    this->current_pid_state = this->None;
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

void fire::drive::set_pid(fire::pid_types pid_type, float kp, float ki, float kd) {
    if (pid_type == fire::pid_types::Drive) {
        this->drive_Kp = kp;
        this->drive_Ki = ki;
        this->drive_Kd = kd;
    } else if (pid_type == fire::pid_types::Turn) {
        this->turn_Kp = kp;
        this->turn_Ki = ki;
        this->turn_Kd = kd;
    }
}

void fire::drive::set_drive_pid(float distance, int speed) {
    this->current_pid_state = pid_state::None;
    
    // set left motor targets
    for (int i = 0; i < this->left_drive.size(); i++) {
        this->left_drive[i].tare_position();
        float start = left_drive[i].get_position();
        float target = (distance/this->diameter)*(float(200)/this->rpm);
        if (left_drive[i].is_reversed()) {
            target = start+target;
        } else {
            target = start-target;
        }
        this->left_targets[i] = target*(float(2)/3);
        this->left_total_error[i] = 0.0;
        this->left_prev_errors[i] = 0.0;
    }

    // set right motor targets
    for (int i = 0; i < this->right_drive.size(); i++) {
        this->right_drive[i].tare_position();
        float start = right_drive[i].get_position();
        float target = (distance/this->diameter)*(float(200)/this->rpm);
        if (right_drive[i].is_reversed()) {
            target = start-target;
        } else {
            target = start+target;
        }
        this->right_targets[i] = target*(float(2)/3);
        this->right_total_error[i] = 0.0;
        this->right_prev_errors[i] = 0.0;
    }

    this->speed = speed;
    this->current_pid_state = pid_state::Drive;
}

void fire::drive::set_turn_pid(float deg, int speed) {
    this->current_pid_state = pid_state::None;

    this->deg_target = deg;
    this->deg_prev_error = 0.0;
    this->deg_total_error = 0.0;

    this->speed = speed;
    this->current_pid_state = pid_state::Turn;
}