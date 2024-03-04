#include "main.h"
#include "autons.h"
#include <string>

fire::drive chassis{
	{1, 3, 5},
	{-11, -12, -13},
	21,
	600,
	1,
	2.75
};

void auton_selector_button() {
	// button to controll selector
	pros::adi::DigitalIn button('d');

	// constantly detect the button
	while (true) {
		// detect if the button is pressed down
		if (button.get_new_press()) {
			// increment the selected auton
			fire::as::page_up();
		}
		// prevent the task from freezing
		pros::delay(fire::delay);
	}
}

void initialize() {
  // set active breaking (broken rn D= )
  // chassis.set_active_breaking(10); // reccomeded to set to 0.1
  // chassis.init_active_breaking();

  // initialize the fire-lib console lcd
  fire::lcd::initialize();

  // calibrate the IMU
  chassis.calibrate_imu();

  // auton selector
  fire::as::add_autons({
      fire::as::Auton("Test Auton", test_auton),
  });
  fire::as::init_selector();

  // clear the display
  // fire::lcd::clear();
  fire::cont.rumble(".");

  // auton selector button; uncomment to enable an adi controlled auton selector
  // pros::Task(auton_selector_button, TASK_PRIORITY_DEFAULT,
  // TASK_STACK_DEPTH_DEFAULT, "Auton Selector Button");
}

void disabled() {
	// stop pid from attempting to run
	chassis.current_pid_state = fire::drive::None;
}

void competition_initialize() {}

void autonomous() {
  fire::as::set_visible(false);
  // necessary if you are using pid loops for accurate movements
  chassis.init_pids();
  // make auton more consistant
  chassis.set_break_mode(pros::v5::MotorBrake::hold);
  // initialize exit conditions for autonomous
  set_exit_conditions();

  // run the auton
  fire::as::call_selected_auton();
}

void opcontrol() {
  fire::as::set_visible(false);
  // fire::lcd::clear();
  chassis.set_break_mode(pros::v5::MotorBrake::coast);
  pros::Controller controller(pros::E_CONTROLLER_MASTER);

  while (true) {
    // drive controlls; uncomment the ONE that you are using
    // chassis.tank_control();
    chassis.split_arcade();
    // chassis.split_arcade_flipped();

	//
	//	ADD DRIVER CODE HERE
	//

    // delay to fix brain freezing
    pros::delay(fire::delay);
  }
}
