#include "robot_hardware.hpp"
#include "main.h"
#include "movetopoint.hpp"
#include "odom.hpp"
#include "auto.h"
#include <cmath>

namespace auton {

void toggle_auton() {
        isAuton = true;

        devices.Motors.MOTOR_SCORE_ROLL.set(127);

        devices.Motors.MOTOR_CASCADE_11WR.set(127);
        devices.Motors.MOTOR_CASCADE_11WL.set(127);
        pros::delay(1000);
        devices.Motors.MOTOR_CASCADE_11WR.set(0);
        devices.Motors.MOTOR_CASCADE_11WL.set(0);

        robot.move(-4, 110, 1000);
        devices.Motors.MOTOR_INTAKE_11W.set(127);
        pros::delay(500);
        devices.Motors.MOTOR_INTAKE_11W.set(0);
        robot.move(5, 110, 1000);
        robot.move(-4, 110, 1000);
        robot.move(5, 110, 1000);

        isAuton = false;
}

void twenty_point_left() {
        isAuton = true;

        toggle_auton();

        robot.move(-5, 127, 1000);
        robot.turn(-90, 127, 1000);

        devices.Motors.MOTOR_SCORE_OPEN.set(10);
        pros::delay(500);
        devices.Motors.MOTOR_SCORE_OPEN.set(0);
        
        robot.move(-10, 127, 2000);

        devices.Motors.MOTOR_SCORE_OPEN.set(10);
        pros::delay(500);
        devices.Motors.MOTOR_SCORE_OPEN.set(0);

        devices.Motors.MOTOR_CASCADE_11WR.set(-127);
        devices.Motors.MOTOR_CASCADE_11WL.set(-127);
        pros::delay(600);
        devices.Motors.MOTOR_CASCADE_11WR.set(0);
        devices.Motors.MOTOR_CASCADE_11WL.set(0);

        devices.Motors.MOTOR_SCORE_ROLL.set(-100);
        pros::delay(500);

        robot.move(5, 127, 1000);

        devices.Motors.MOTOR_SCORE_ROLL.set(0);

        devices.Motors.MOTOR_CASCADE_11WR.set(-127);
        devices.Motors.MOTOR_CASCADE_11WL.set(-127);
        pros::delay(400);
        devices.Motors.MOTOR_CASCADE_11WR.set(0);
        devices.Motors.MOTOR_CASCADE_11WL.set(0);

        isAuton = false;
}

void twenty_point_right() {
        isAuton = true;

        toggle_auton();

        robot.move(-5, 127, 1000);
        robot.turn(90, 127, 1000);

        devices.Motors.MOTOR_SCORE_OPEN.set(10);
        pros::delay(500);
        devices.Motors.MOTOR_SCORE_OPEN.set(0);
        
        robot.move(-8, 127, 2000);

        devices.Motors.MOTOR_SCORE_OPEN.set(10);
        pros::delay(500);
        devices.Motors.MOTOR_SCORE_OPEN.set(0);

        devices.Motors.MOTOR_CASCADE_11WR.set(-127);
        devices.Motors.MOTOR_CASCADE_11WL.set(-127);
        pros::delay(600);
        devices.Motors.MOTOR_CASCADE_11WR.set(0);
        devices.Motors.MOTOR_CASCADE_11WL.set(0);

        devices.Motors.MOTOR_SCORE_ROLL.set(-100);
        pros::delay(500);

        robot.move(5, 127, 1000);

        devices.Motors.MOTOR_SCORE_ROLL.set(0);

        isAuton = false;
}

void middle_pin_left() {
        isAuton = true;
        

        
        isAuton = false;
}

void middle_pin_right() {
        isAuton = true;

        isAuton = false;
}

void thirty_point_left() {
        isAuton = true;

        twenty_point_left();

        robot.turn(58, 127, 1000);

        devices.Motors.MOTOR_SCORE_ROLL.set(127);

        devices.Motors.MOTOR_SCORE_OPEN.set(-10);
        pros::delay(500);
        devices.Motors.MOTOR_SCORE_OPEN.set(0);

        robot.move(-11, 90, 1000);

        pros::delay(500);

        robot.turn(-125, 127, 1000);

        devices.Motors.MOTOR_CASCADE_11WR.set(127);
        devices.Motors.MOTOR_CASCADE_11WL.set(127);
        pros::delay(2000);
        devices.Motors.MOTOR_CASCADE_11WR.set(0);
        devices.Motors.MOTOR_CASCADE_11WL.set(0);

        robot.move(-6, 127, 1000);

        devices.Motors.MOTOR_SCORE_OPEN.set(30);
        pros::delay(500);
        devices.Motors.MOTOR_SCORE_OPEN.set(0);

        devices.Motors.MOTOR_CASCADE_11WR.set(-127);
        devices.Motors.MOTOR_CASCADE_11WL.set(-127);
        pros::delay(600);
        devices.Motors.MOTOR_CASCADE_11WR.set(0);
        devices.Motors.MOTOR_CASCADE_11WL.set(0);


        devices.Motors.MOTOR_SCORE_ROLL.set(-127);

        robot.move(2, 127, 1000);


        isAuton = false;
}

void thirty_point_right() {
        isAuton = true;

        toggle_auton();

        robot.move(-5, 127, 1000);
        robot.turn(90, 127, 1000);
        devices.Motors.MOTOR_SCORE_OPEN.set(60);
        pros::delay(500);
        devices.Motors.MOTOR_SCORE_OPEN.set(0);

        robot.move(-12, 80, 2000);

        devices.Motors.MOTOR_SCORE_OPEN.set(-10);
        pros::delay(300);
        devices.Motors.MOTOR_SCORE_OPEN.set(0);

        devices.Motors.MOTOR_SCORE_ROLL.set(-50);
        pros::delay(1000);
        devices.Motors.MOTOR_SCORE_ROLL.set(0);

        devices.Motors.MOTOR_SCORE_OPEN.set(20);
        pros::delay(500);
        devices.Motors.MOTOR_SCORE_OPEN.set(0);

        robot.move(5, 127, 1000);

        robot.turn(-62, 127, 1000);

        devices.Motors.MOTOR_SCORE_OPEN.set(40);
        pros::delay(500);
        devices.Motors.MOTOR_SCORE_OPEN.set(0);

        devices.Motors.MOTOR_SCORE_ROLL.set(127);
        
        robot.move(-7.8, 120, 1000);

        devices.Motors.MOTOR_SCORE_OPEN.set(-40);
        pros::delay(500);
        devices.Motors.MOTOR_SCORE_OPEN.set(0);

        pros::delay(500);

        devices.Motors.MOTOR_SCORE_OPEN.set(40);
        pros::delay(700);
        devices.Motors.MOTOR_SCORE_OPEN.set(0);

        robot.turn(110, 127, 1000);

        devices.Motors.MOTOR_CASCADE_11WR.set(127);
        devices.Motors.MOTOR_CASCADE_11WL.set(127);
        pros::delay(500);
        devices.Motors.MOTOR_CASCADE_11WR.set(0);
        devices.Motors.MOTOR_CASCADE_11WL.set(0);

        robot.move(-7, 80, 1000);

        devices.Motors.MOTOR_SCORE_OPEN.set(-20);
        pros::delay(450);
        devices.Motors.MOTOR_SCORE_OPEN.set(0);

        devices.Motors.MOTOR_SCORE_ROLL.set(-60);

        pros::delay(1000);

        devices.Motors.MOTOR_SCORE_OPEN.set(50);
        pros::delay(450);
        devices.Motors.MOTOR_SCORE_OPEN.set(0);

        robot.move(2, 127, 1000);


        isAuton = false;
}

void forty_point_left() {
        isAuton = true;

        thirty_point_left();

        robot.move(6, 127, 2000);

        robot.turn(-45, 127, 1000);

        devices.Motors.MOTOR_SCORE_ROLL.set(127);

        robot.move(-4, 127, 2000);

        //add other stuff lmao

        isAuton = false;
}

void forty_point_right() {
        isAuton = true;

        thirty_point_right();

        

        

        isAuton = false;
}

void skills_routine() {
        isAuton = true; //need to add park to skills and try for a 30 point run (2 line pins)

        thirty_point_left();

        robot.move(8, 127, 2000);

        isAuton = false;
}

void recorded_routine() {
    isAuton = true;

    isAuton = false;
}

void nodrive() {
        isAuton = true;

        isAuton = false;
}

} // namespace