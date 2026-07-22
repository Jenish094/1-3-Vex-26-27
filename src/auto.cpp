#include "auto.h"
#include "robotconfig.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "pros/rtos.hpp"

ASSET(redfront_txt);

void redFront() {
	chassis.arcade(-45, 0, false, 0.25);
	pros::delay(300);
	chassis.arcade(0, 0, false, 0.25);

	Lift.move(127);
	pros::delay(2000);
	Lift.move(-127);
	pros::delay(2000);
	Lift.move(0);

	chassis.arcade(127, 0, false, 0.25);
	pros::delay(250);
	chassis.arcade(0, 0, false, 0.25);

}

void redLeft() {
    	chassis.arcade(-45, 0, false, 0.25);
	pros::delay(300);
	chassis.arcade(0, 0, false, 0.25);

	Lift.move(127);
	pros::delay(2000);
	Lift.move(-127);
	pros::delay(2000);
	Lift.move(0);

	chassis.arcade(127, 0, false, 0.25);
	pros::delay(250);
	chassis.arcade(0, 0, false, 0.25);
}

void blueFront() {
    	chassis.arcade(-45, 0, false, 0.25);
	pros::delay(300);
	chassis.arcade(0, 0, false, 0.25);

	Lift.move(127);
	pros::delay(2000);
	Lift.move(-127);
	pros::delay(2000);
	Lift.move(0);

	chassis.arcade(127, 0, false, 0.25);
	pros::delay(250);
	chassis.arcade(0, 0, false, 0.25);
}

void blueLeft() {
    	chassis.arcade(-45, 0, false, 0.25);
	pros::delay(300);
	chassis.arcade(0, 0, false, 0.25);

	Lift.move(127);
	pros::delay(2000);
	Lift.move(-127);
	pros::delay(2000);
	Lift.move(0);

	chassis.arcade(127, 0, false, 0.25);
	pros::delay(250);
	chassis.arcade(0, 0, false, 0.25);
}

void skills_auton() {
}
