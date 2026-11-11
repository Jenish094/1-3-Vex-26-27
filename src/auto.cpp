#include "robot_hardware.hpp"
#include "main.h"
#include "movetopoint.hpp"
#include "odom.hpp"
#include "auto.h"
#include <cmath>

namespace auton {

void toggle_auton() {
        isAuton = true;

        robot.move(-4, 110, 1000);
        robot.move(6, 110, 1000);
        robot.move(-4, 110, 1000);
        robot.move(6, 110, 1000);

        isAuton = false;
}

void one_pin_left() {
        isAuton = true;

        toggle_auton();

        robot.move(-6, 120, 1000);
        robot.turn(-90, 120, 1000);
        devices.Motors.MOTOR_SCORE_OPEN.set(60);
        pros::delay(500);
        devices.Motors.MOTOR_SCORE_OPEN.set(0);

        robot.move(-12, 120, 1000);

        devices.Motors.MOTOR_SCORE_OPEN.set(-10);
        pros::delay(300);
        devices.Motors.MOTOR_SCORE_OPEN.set(0);

        devices.Motors.MOTOR_SCORE_ROLL.set(-50);
        pros::delay(1000);
        devices.Motors.MOTOR_SCORE_ROLL.set(0);

        devices.Motors.MOTOR_SCORE_OPEN.set(20);
        pros::delay(500);
        devices.Motors.MOTOR_SCORE_OPEN.set(0);

        robot.move(5, 120, 1000);

        robot.turn(60, 120, 1000);

        devices.Motors.MOTOR_SCORE_OPEN.set(40);
        pros::delay(500);
        devices.Motors.MOTOR_SCORE_OPEN.set(0);

        devices.Motors.MOTOR_SCORE_ROLL.set(127);
        
        robot.move(-9.3, 120, 1000);

        devices.Motors.MOTOR_SCORE_OPEN.set(-40);
        pros::delay(500);
        devices.Motors.MOTOR_SCORE_OPEN.set(0);

        pros::delay(500);

        devices.Motors.MOTOR_SCORE_OPEN.set(40);
        pros::delay(500);
        devices.Motors.MOTOR_SCORE_OPEN.set(0);

        robot.move(10, 120, 1000);

        devices.Motors.MOTOR_SCORE_ROLL.set(0);

        robot.turn(-60, 120, 1000);

        devices.Motors.MOTOR_CASCADE_11WR.set(127);
        devices.Motors.MOTOR_CASCADE_11WL.set(127);
        pros::delay(700);
        devices.Motors.MOTOR_CASCADE_11WR.set(0);
        devices.Motors.MOTOR_CASCADE_11WL.set(0);

        robot.move(-12, 100, 1000);

        devices.Motors.MOTOR_SCORE_OPEN.set(-30);
        pros::delay(450);
        devices.Motors.MOTOR_SCORE_OPEN.set(0);

        devices.Motors.MOTOR_SCORE_ROLL.set(-60);

        pros::delay(1000);

        devices.Motors.MOTOR_SCORE_OPEN.set(50);
        pros::delay(450);
        devices.Motors.MOTOR_SCORE_OPEN.set(0);

        robot.move(5, 120, 1000);




        isAuton = false;
}

void one_pin_right() {
        isAuton = true;

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

        one_pin_left();

        isAuton = false;
}

void thirty_point_right() {
        isAuton = true;

        one_pin_right();

        isAuton = false;
}

void forty_point_left() {
        isAuton = true;

        isAuton = false;
}

void forty_point_right() {
        isAuton = true;

        isAuton = false;
}

void skills_routine() {
        isAuton = true;

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