#include "main.h"
#include "fire.h"

fire::drive chassis{
	{},
	{},
};

void initialize() {
	// initialize the fire-lib console lcd
	fire::lcd::initialize();
}

void disabled() {}

void competition_initialize() {}

void autonomous() {}

void opcontrol() {
	pros::Controller controller(pros::E_CONTROLLER_MASTER);

	while (true) {
		// drive controlls. uncomment the ONE that you are using
		//chassis.tank_control();
		chassis.split_arcade();
		//chassis.split_arcade_flipped();

		// delay to fix brain freezing
		pros::delay(fire::delay);
	}
}