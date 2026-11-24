#include "robot_hardware.hpp"
#include "main.h"
#include "movetopoint.hpp"
#include "odom.hpp"
#include "auto.h"
#include <cmath>

namespace auton {

namespace {

constexpr int kTwentyPointLeftTurn = 90;
constexpr int kThirtyPointLeftTurn = 58;
constexpr int kThirtyPointLeftReturnTurn = -125;
constexpr int kFortyPointLeftTurn = 55;
constexpr int kFortyPointLeftReturnTurn = -90;

int turn_for_side(int leftAngle, bool rightSide) {
        return rightSide ? -leftAngle : leftAngle;
}

void run_twenty_point_sequence(bool rightSide) {
        isAuton = true;

        toggle_auton();

        robot.move(-5, 127, 1000);
        robot.turn(turn_for_side(kTwentyPointLeftTurn, rightSide), 127, 1000);

        devices.Motors.MOTOR_SCORE_OPEN.set(10);
        pros::delay(500);
        devices.Motors.MOTOR_SCORE_OPEN.set(0);
        
        robot.move(-8, 80, 2000);

        devices.Motors.MOTOR_CASCADE_11WR.set(-127);
        devices.Motors.MOTOR_CASCADE_11WL.set(-127);
        devices.Motors.MOTOR_SCORE_OPEN.set(10);

        pros::delay(500);
        devices.Motors.MOTOR_SCORE_OPEN.set(0);

        pros::delay(100);
        devices.Motors.MOTOR_CASCADE_11WR.set(0);
        devices.Motors.MOTOR_CASCADE_11WL.set(0);        

        devices.Motors.MOTOR_SCORE_ROLL.set(-100);
        pros::delay(500);

        robot.move(5, 127, 1000);

        devices.Motors.MOTOR_SCORE_ROLL.set(0);

        devices.Motors.MOTOR_CASCADE_11WR.set(-127);
        devices.Motors.MOTOR_CASCADE_11WL.set(-127);
        pros::delay(900);
        devices.Motors.MOTOR_CASCADE_11WR.set(0);
        devices.Motors.MOTOR_CASCADE_11WL.set(0);

        isAuton = false;
}

void run_thirty_point_sequence(bool rightSide) {
        isAuton = true;

        run_twenty_point_sequence(rightSide);

        robot.turn(turn_for_side(kThirtyPointLeftTurn, rightSide), 127, 1000);

        //check lines 136 - 144 idk if it will work but it should, if not just uncomment the lines and delete the task

        devices.Motors.MOTOR_SCORE_ROLL.set(127);

        pros::Task scoreOpenTimer([](void*) {
                devices.Motors.MOTOR_SCORE_OPEN.set(-10);
                pros::delay(500);
                devices.Motors.MOTOR_SCORE_OPEN.set(0);
        });

        robot.move(-11, 90, 1000);

        pros::delay(500);

        devices.Motors.MOTOR_CASCADE_11WR.set(127);
        devices.Motors.MOTOR_CASCADE_11WL.set(127);
        devices.Motors.MOTOR_SCORE_OPEN.set(30);

        pros::delay(500);
        devices.Motors.MOTOR_SCORE_OPEN.set(0);

        pros::delay(1500);
        devices.Motors.MOTOR_CASCADE_11WR.set(0);
        devices.Motors.MOTOR_CASCADE_11WL.set(0);

        robot.turn(turn_for_side(kThirtyPointLeftReturnTurn, rightSide), 127, 1000);

        robot.move(-6, 127, 1000);

        devices.Motors.MOTOR_CASCADE_11WR.set(-127);
        devices.Motors.MOTOR_CASCADE_11WL.set(-127);
        pros::delay(600);
        devices.Motors.MOTOR_CASCADE_11WR.set(0);
        devices.Motors.MOTOR_CASCADE_11WL.set(0);

        devices.Motors.MOTOR_SCORE_ROLL.set(-127);

        robot.move(2, 127, 1000);

        devices.Motors.MOTOR_CASCADE_11WR.set(-127);
        devices.Motors.MOTOR_CASCADE_11WL.set(-127);
        pros::delay(1400);
        devices.Motors.MOTOR_CASCADE_11WR.set(0);
        devices.Motors.MOTOR_CASCADE_11WL.set(0);

        isAuton = false;
}

void run_forty_point_sequence(bool rightSide) {
        isAuton = true;

        run_thirty_point_sequence(rightSide);

        robot.move(3, 127, 2000);

        robot.turn(turn_for_side(kFortyPointLeftTurn, rightSide), 127, 1000);

        devices.Motors.MOTOR_SCORE_OPEN.set(-30);
        pros::delay(400);
        devices.Motors.MOTOR_SCORE_OPEN.set(0);

        devices.Motors.MOTOR_SCORE_ROLL.set(127);

        robot.move(-10, 127, 2000);

        devices.Motors.MOTOR_CASCADE_11WR.set(127);
        devices.Motors.MOTOR_CASCADE_11WL.set(127);
        devices.Motors.MOTOR_SCORE_OPEN.set(30);

        pros::delay(500);
        devices.Motors.MOTOR_SCORE_OPEN.set(0);

        pros::delay(2800);
        devices.Motors.MOTOR_CASCADE_11WR.set(0);
        devices.Motors.MOTOR_CASCADE_11WL.set(0);

        robot.turn(turn_for_side(kFortyPointLeftReturnTurn, rightSide), 100, 1000);

        robot.move(-5, 100, 1000);

        devices.Motors.MOTOR_SCORE_ROLL.set(-127);

        isAuton = false;
}

} // namespace

void toggle_auton() {
        isAuton = true;

        devices.Motors.MOTOR_SCORE_ROLL.set(127);

        // Start the cascade, then immediately begin moving. Both commands run
        // concurrently because the motors remain powered while robot.move waits.
        devices.Motors.MOTOR_CASCADE_11WR.set(127);
        devices.Motors.MOTOR_CASCADE_11WL.set(127);
        const std::uint32_t cascadeStart = pros::millis();
        robot.move(-4, 110, 1000);
        const std::uint32_t cascadeElapsed = pros::millis() - cascadeStart;
        if (cascadeElapsed < 1500) {
                pros::delay(1500 - cascadeElapsed);
        }
        devices.Motors.MOTOR_CASCADE_11WR.set(0);
        devices.Motors.MOTOR_CASCADE_11WL.set(0);

        devices.Motors.MOTOR_INTAKE_11W.set(127);
        pros::delay(500);
        devices.Motors.MOTOR_INTAKE_11W.set(0);
        robot.move(5, 110, 1000);
        robot.move(-4, 110, 1000);
        robot.move(5, 110, 1000);

        isAuton = false;
}

void twenty_point_left() {
        run_twenty_point_sequence(false);
}

void twenty_point_right() {
        run_twenty_point_sequence(true);
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
        run_thirty_point_sequence(false);
}

void thirty_point_right() {
        run_thirty_point_sequence(true);
}

void forty_point_left() {
        run_forty_point_sequence(false);
}

void forty_point_right() {
        run_forty_point_sequence(true);
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