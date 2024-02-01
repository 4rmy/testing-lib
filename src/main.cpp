#include "main.h"
#include "autons.h"
#include <cstddef>
#include <string>

fire::drive chassis{
	{-1, -3, -5},
	{11, 12, 13},
	21,
	600,
	float(36)/60,
	3.25
};

void initialize() {
	// initialize the fire-lib console lcd
	fire::lcd::initialize();
	fire::lcd::print("Calabrating IMU...");
	chassis.imu.reset(true);
	fire::lcd::print("IMU Calabrated.");

	// initialize exit conditions for autonomous
	set_exit_conditions();
	// clear the display
	fire::lcd::clear();
	fire::cont.rumble(".");
}

void disabled() {}

void competition_initialize() {}

void autonomous() {
	// necessary if you are using pid loops for accurate movements
	chassis.init_pids();

	test_auton();
}

void opcontrol() {
	pros::Controller controller(pros::E_CONTROLLER_MASTER);
	//pros::Motor test(12);

	while (true) {
		fire::lcd::println(0, "IMU Rotation: " + std::to_string(chassis.imu.get_rotation()));
		// drive controlls. uncomment the ONE that you are using
		//chassis.tank_control();
		chassis.split_arcade();
		//chassis.split_arcade_flipped();

		if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
			autonomous();
		}

		// delay to fix brain freezing
		pros::delay(fire::delay);
	}
}
