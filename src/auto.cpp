#include "hardware.hpp"
#include "main.h"
#include "movetopoint.hpp"
#include "odom.hpp"
#include "auto.h"
#include <cmath>

namespace auton {

// Need to check all ms delay timings for movement 45, 90, 180
void fourtyfivedegleft() {
        devices.MGroups.DT_L.set(-60);
        devices.MGroups.DT_R.set(60);
        pros::delay(380);
        devices.MGroups.DT_L.set(0);
        devices.MGroups.DT_R.set(0);

}

void ninetydegleft() {
        fourtyfivedegleft();
        pros::delay(1);
        fourtyfivedegleft();
}

void fourtyfivedegright() {
        devices.MGroups.DT_L.set(60);
        devices.MGroups.DT_R.set(-60);
        pros::delay(380);
        devices.MGroups.DT_L.set(0);
        devices.MGroups.DT_R.set(0);
}

void ninetydegright() {
        fourtyfivedegright();
        pros::delay(1);
        fourtyfivedegright();
}

void oneeighty() {
        ninetydegleft();
        pros::delay(1);
        ninetydegleft();
}

void onetwentyleft(){
        left();
        pros::delay(1000);
        stop();
}

void onetwentyright(){
        right();
        pros::delay(1000);
        stop();
}

void forward() {
        devices.MGroups.DT_L.set(60);
        devices.MGroups.DT_R.set(60);
}

void half_speed_forward(){
        devices.MGroups.DT_L.set(30);
        devices.MGroups.DT_R.set(30);
}

void backwards() {
        devices.MGroups.DT_L.set(-60);
        devices.MGroups.DT_R.set(-60);
}

void half_speed_backwards(){
        devices.MGroups.DT_L.set(-30);
        devices.MGroups.DT_R.set(-30);
}

void left() {
        devices.MGroups.DT_L.set(-60);
        devices.MGroups.DT_R.set(60);
}

void half_speed_left() {
        devices.MGroups.DT_L.set(-30);
        devices.MGroups.DT_R.set(30);
}

void right() {
        devices.MGroups.DT_L.set(60);
        devices.MGroups.DT_R.set(-60);
}

void half_speed_right() {
        devices.MGroups.DT_L.set(30);
        devices.MGroups.DT_R.set(-30);
}

void stop() {
        devices.MGroups.DT_L.set(0);
        devices.MGroups.DT_R.set(0);
}



void test(){
        isAuton = true;

        startOdometryTask();
        driveInches(12.0, 75.0, 4000.0);

        const double targetHeading =
            static_cast<double>(globalHeading.load()) + (ODOM_PI * 0.5);
        const std::uint32_t turnStart = pros::millis();
        const std::uint32_t turnTimeoutMs = 3000;
        const std::uint32_t settleTimeMs = 150;
        const double headingTolerance = 2.0 * ODOM_PI / 180.0;
        std::uint32_t settledSince = 0;
        bool settling = false;

        while (pros::millis() - turnStart < turnTimeoutMs) {
                const double headingError = std::remainder(
                    targetHeading - static_cast<double>(globalHeading.load()),
                    2.0 * ODOM_PI);

                if (std::abs(headingError) <= headingTolerance) {
                        if (!settling) {
                                settledSince = pros::millis();
                                settling = true;
                        } else if (pros::millis() - settledSince >= settleTimeMs) {
                                break;
                        }
                        devices.MGroups.DT_L.set(0);
                        devices.MGroups.DT_R.set(0);
                } else {
                        settling = false;
                        double turnPower = std::clamp(headingError * 70.0, -60.0, 60.0);
                        if (std::abs(turnPower) < 12.0)
                                turnPower = std::copysign(12.0, turnPower);

                        // Positive odometry heading is a left (counterclockwise) turn.
                        devices.MGroups.DT_L.set(static_cast<int>(std::lround(-turnPower)));
                        devices.MGroups.DT_R.set(static_cast<int>(std::lround(turnPower)));
                }

                pros::delay(10);
        }

        devices.MGroups.DT_L.set(0);
        devices.MGroups.DT_R.set(0);
        isAuton = false;
}

void toggle_auton(){
        isAuton = true;

        backwards();
        pros::delay(300);
        stop();

        // Open scoring pneumatic / intake action
        devices.MGroups.MOTORGROUP_INTAKE.set(127);
        pros::delay(500);
        devices.MGroups.MOTORGROUP_INTAKE.set(0);

        // Move robot back at double speed
        devices.MGroups.DT_L.set(120);
        devices.MGroups.DT_R.set(120);
        pros::delay(500);
        devices.MGroups.DT_L.set(0);
        devices.MGroups.DT_R.set(0);

        pros::delay(1000);

        // Run the second toggle
        // Move robot slightly backwards
        backwards();
        pros::delay(600);
        stop();

        // Move robot back at double speed
        forward();
        pros::delay(600);
        stop();

        // Move robot slightly forward
        forward();
        pros::delay(100);
        stop();

        isAuton = false;
}

void left_side_auton(){
        isAuton = true;

        toggle_auton();

        pros::delay(200);

        //turn left to score in goal
        left();
        pros::delay(580);
        stop();

        devices.Motors.MOTOR_SCORE_OPEN.set(40);
        pros::delay(350);
        stop();

        half_speed_backwards();
        pros::delay(2000);
        stop();

        pros::delay(200);

        devices.Motors.MOTOR_SCORE_OPEN.set(-30);
        pros::delay(350);
        stop();

        devices.Motors.MOTOR_SCORE_ROLL.set(-60);
        pros::delay(500);
        stop();
        
        devices.Motors.MOTOR_SCORE_OPEN.set(40);
        pros::delay(150);
        stop();

        //pin now in goal, move back to get out of goal

        forward();
        pros::delay(300);
        stop();

        fourtyfivedegleft();
        pros::delay(1000);

        forward();
        pros::delay(400);
        stop();

        // fourtyfivedegright();
        // pros::delay(1000);

        devices.Motors.MOTOR_SCORE_ROLL.set(0);
        pros::delay(1);
        stop();

        right();
        pros::delay(510);
        stop();

        devices.Motors.MOTOR_SCORE_ROLL.set(60);
        pros::delay(100);
        
        half_speed_backwards();
        pros::delay(2200);
        stop();

        //bacakwards towards pin #1

        devices.Motors.MOTOR_SCORE_OPEN.set(-40);
        pros::delay(450);
        stop();

        //moves bancee down to intake pin off ground

        devices.Motors.MOTOR_SCORE_OPEN.set(30);
        pros::delay(200);
        stop();

        forward();
        pros::delay(500);
        stop();

        left();
        pros::delay(650);
        stop();
        //turned to face the goal

        backwards();
        pros::delay(1000);
        stop();

        devices.Motors.MOTOR_SCORE_OPEN.set(-30);
        pros::delay(400);
        stop();

        devices.Motors.MOTOR_SCORE_ROLL.set(-60);
        pros::delay(100);

        devices.Motors.MOTOR_SCORE_OPEN.set(60);
        pros::delay(400);
        stop();

        pros::delay(200);

        devices.Motors.MOTOR_CASCADE_11WL.set(127);
        devices.Motors.MOTOR_CASCADE_11WR.set(127);
        pros::delay(700);
        devices.Motors.MOTOR_CASCADE_11WL.set(0);
        devices.Motors.MOTOR_CASCADE_11WR.set(0);
        stop();

        forward();
        pros::delay(200);
        stop();


        devices.Motors.MOTOR_SCORE_ROLL.set(-10);
        pros::delay(100);
        stop();


        //first pin dropped off, now going for second pin and cup


        forward();
        pros::delay(500);
        stop();

        devices.Motors.MOTOR_CASCADE_11WL.set(-127);
        devices.Motors.MOTOR_CASCADE_11WR.set(-127);
        pros::delay(700);
        devices.Motors.MOTOR_CASCADE_11WL.set(0);
        devices.Motors.MOTOR_CASCADE_11WR.set(0);
        stop();

        devices.Motors.MOTOR_SCORE_OPEN.set(0);
        pros::delay(1);
        stop();

        right();
        pros::delay(650);
        stop();

        backwards();
        pros::delay(1000);
        stop();

        left();
        pros::delay(710);
        stop();

        backwards();
        pros::delay(1400);
        stop();

        devices.Motors.MOTOR_SCORE_ROLL.set(40);
        pros::delay(100);

        devices.Motors.MOTOR_SCORE_OPEN.set(-30);
        pros::delay(400);
        stop();

        devices.Motors.MOTOR_SCORE_OPEN.set(30);
        pros::delay(300);
        stop();

        forward();
        pros::delay(800);
        stop();

        left();
        pros::delay(580);
        stop();

        backwards();
        pros::delay(700);
        stop();



        isAuton = false;
}

void left_side_auton_middle_pin(){
        isAuton = true;

        toggle_auton();

        pros::delay(200);

        //turn left to score in goal
        left();
        pros::delay(580);
        stop();

        devices.Motors.MOTOR_SCORE_OPEN.set(40);
        pros::delay(350);
        stop();

        backwards();
        pros::delay(1000);
        stop();

        pros::delay(200);

        devices.Motors.MOTOR_SCORE_OPEN.set(-30);
        pros::delay(350);
        stop();

        devices.Motors.MOTOR_SCORE_ROLL.set(-60);
        pros::delay(500);
        stop();
        
        devices.Motors.MOTOR_SCORE_OPEN.set(40);
        pros::delay(150);
        stop();

        //pin now in goal, move back to get out of goal

        forward();
        pros::delay(300);
        stop();

        fourtyfivedegleft();
        pros::delay(1000);

        forward();
        pros::delay(400);
        stop();

        right();
        pros::delay(850);
        stop();

        devices.Motors.MOTOR_SCORE_OPEN.set(30);
        pros::delay(200);
        stop();

        devices.Motors.MOTOR_SCORE_ROLL.set(60);
        pros::delay(100);

        backwards();
        pros::delay(700);
        stop();

        devices.Motors.MOTOR_SCORE_OPEN.set(-30);
        pros::delay(600);
        stop();

        devices.Motors.MOTOR_SCORE_OPEN.set(60);
        pros::delay(100);
        stop();

        devices.Motors.MOTOR_CASCADE_11WL.set(-127);
        devices.Motors.MOTOR_CASCADE_11WR.set(-127);
        pros::delay(1000);
        devices.Motors.MOTOR_CASCADE_11WL.set(0);
        devices.Motors.MOTOR_CASCADE_11WR.set(0);
        stop();

        backwards();
        pros::delay(1100);
        stop();

        pros::delay(500);

        devices.Motors.MOTOR_SCORE_OPEN.set(-10);
        pros::delay(400);
        stop();

        devices.Motors.MOTOR_SCORE_ROLL.set(-60);
        pros::delay(100);
        
        devices.Motors.MOTOR_CASCADE_11WL.set(-127);
        devices.Motors.MOTOR_CASCADE_11WR.set(-127);
        pros::delay(1300);
        devices.Motors.MOTOR_CASCADE_11WL.set(0);
        devices.Motors.MOTOR_CASCADE_11WR.set(0);
        stop();

        forward();
        pros::delay(200);
        stop();

        devices.Motors.MOTOR_CASCADE_11WL.set(127);
        devices.Motors.MOTOR_CASCADE_11WR.set(127);
        pros::delay(2000);
        devices.Motors.MOTOR_CASCADE_11WL.set(0);
        devices.Motors.MOTOR_CASCADE_11WR.set(0);
        
        isAuton = false;
}

void right_side_auton(){
        isAuton = true;

        toggle_auton();

        pros::delay(200);

        //turn right to score in goal
        right();
        pros::delay(580);
        stop();

        devices.Motors.MOTOR_SCORE_OPEN.set(40);
        pros::delay(350);
        stop();

        half_speed_backwards();
        pros::delay(2000);
        stop();

        pros::delay(200);

        devices.Motors.MOTOR_SCORE_OPEN.set(-30);
        pros::delay(350);
        stop();

        devices.Motors.MOTOR_SCORE_ROLL.set(-60);
        pros::delay(500);
        stop();
        
        devices.Motors.MOTOR_SCORE_OPEN.set(40);
        pros::delay(150);
        stop();

        //pin now in goal, move back to get out of goal

        forward();
        pros::delay(300);
        stop();

        fourtyfivedegleft();
        pros::delay(1000);

        forward();
        pros::delay(400);
        stop();

        // fourtyfivedegright();
        // pros::delay(1000);

        devices.Motors.MOTOR_SCORE_ROLL.set(0);
        pros::delay(1);
        stop();

        left();
        pros::delay(510);
        stop();

        devices.Motors.MOTOR_SCORE_ROLL.set(60);
        pros::delay(100);
        
        backwards();
        pros::delay(1100);
        stop();

        //bacakwards towards pin #1

        devices.Motors.MOTOR_SCORE_OPEN.set(-40);
        pros::delay(450);
        stop();

        //moves bancee down to intake pin off ground

        devices.Motors.MOTOR_SCORE_OPEN.set(30);
        pros::delay(200);
        stop();

        forward();
        pros::delay(500);
        stop();

        right();
        pros::delay(650);
        stop();
        //turned to face the goal

        backwards();
        pros::delay(1000);
        stop();

        devices.Motors.MOTOR_SCORE_OPEN.set(-30);
        pros::delay(400);
        stop();

        devices.Motors.MOTOR_SCORE_ROLL.set(-60);
        pros::delay(100);

        devices.Motors.MOTOR_SCORE_OPEN.set(60);
        pros::delay(400);
        stop();

        pros::delay(200);

        devices.Motors.MOTOR_CASCADE_11WL.set(127);
        devices.Motors.MOTOR_CASCADE_11WR.set(127);
        pros::delay(700);
        devices.Motors.MOTOR_CASCADE_11WL.set(0);
        devices.Motors.MOTOR_CASCADE_11WR.set(0);
        stop();

        forward();
        pros::delay(200);
        stop();


        devices.Motors.MOTOR_SCORE_ROLL.set(-10);
        pros::delay(100);
        stop();


        //first pin dropped off, now going for second pin and cup


        forward();
        pros::delay(500);
        stop();

        devices.Motors.MOTOR_CASCADE_11WL.set(-127);
        devices.Motors.MOTOR_CASCADE_11WR.set(-127);
        pros::delay(700);
        devices.Motors.MOTOR_CASCADE_11WL.set(0);
        devices.Motors.MOTOR_CASCADE_11WR.set(0);
        stop();

        devices.Motors.MOTOR_SCORE_OPEN.set(0);
        pros::delay(1);
        stop();

        left();
        pros::delay(650);
        stop();

        backwards();
        pros::delay(1000);
        stop();

        right();
        pros::delay(710);
        stop();

        backwards();
        pros::delay(1400);
        stop();

        devices.Motors.MOTOR_SCORE_ROLL.set(40);
        pros::delay(100);

        devices.Motors.MOTOR_SCORE_OPEN.set(-30);
        pros::delay(400);
        stop();

        devices.Motors.MOTOR_SCORE_OPEN.set(30);
        pros::delay(300);
        stop();

        forward();
        pros::delay(800);
        stop();

        right();
        pros::delay(580);
        stop();

        backwards();
        pros::delay(700);
        stop();

        isAuton = false;
}

void right_side_auton_middle_pin(){
    isAuton = true;

        toggle_auton();

        pros::delay(200);

        //turn left to score in goal
        right();
        pros::delay(580);
        stop();

        devices.Motors.MOTOR_SCORE_OPEN.set(40);
        pros::delay(350);
        stop();

        backwards();
        pros::delay(1000);
        stop();

        pros::delay(200);

        devices.Motors.MOTOR_SCORE_OPEN.set(-30);
        pros::delay(350);
        stop();

        devices.Motors.MOTOR_SCORE_ROLL.set(-60);
        pros::delay(500);
        stop();
        
        devices.Motors.MOTOR_SCORE_OPEN.set(40);
        pros::delay(150);
        stop();

        //pin now in goal, move back to get out of goal

        forward();
        pros::delay(300);
        stop();

        fourtyfivedegleft();
        pros::delay(1000);

        forward();
        pros::delay(400);
        stop();

        left();
        pros::delay(850);
        stop();

        devices.Motors.MOTOR_SCORE_OPEN.set(30);
        pros::delay(200);
        stop();

        devices.Motors.MOTOR_SCORE_ROLL.set(60);
        pros::delay(100);

        backwards();
        pros::delay(700);
        stop();

        devices.Motors.MOTOR_SCORE_OPEN.set(-30);
        pros::delay(600);
        stop();

        devices.Motors.MOTOR_SCORE_OPEN.set(60);
        pros::delay(100);
        stop();

        devices.Motors.MOTOR_CASCADE_11WL.set(-127);
        devices.Motors.MOTOR_CASCADE_11WR.set(-127);
        pros::delay(1000);
        devices.Motors.MOTOR_CASCADE_11WL.set(0);
        devices.Motors.MOTOR_CASCADE_11WR.set(0);
        stop();

        backwards();
        pros::delay(1100);
        stop();

        pros::delay(500);

        devices.Motors.MOTOR_SCORE_OPEN.set(-10);
        pros::delay(400);
        stop();

        devices.Motors.MOTOR_SCORE_ROLL.set(-60);
        pros::delay(100);
        
        devices.Motors.MOTOR_CASCADE_11WL.set(-127);
        devices.Motors.MOTOR_CASCADE_11WR.set(-127);
        pros::delay(1300);
        devices.Motors.MOTOR_CASCADE_11WL.set(0);
        devices.Motors.MOTOR_CASCADE_11WR.set(0);
        stop();

        forward();
        pros::delay(200);
        stop();

        devices.Motors.MOTOR_CASCADE_11WL.set(127);
        devices.Motors.MOTOR_CASCADE_11WR.set(127);
        pros::delay(2000);
        devices.Motors.MOTOR_CASCADE_11WL.set(0);
        devices.Motors.MOTOR_CASCADE_11WR.set(0);
        
        isAuton = false;    
}

void left_side_toggle_only(){
        isAuton = true;

        toggle_auton();

        pros::delay(200);

        //turn left to score in goal
        left();
        pros::delay(580);
        stop();

        devices.Motors.MOTOR_SCORE_OPEN.set(40);
        pros::delay(350);
        stop();

        backwards();
        pros::delay(1000);
        stop();

        pros::delay(200);

        devices.Motors.MOTOR_SCORE_OPEN.set(-30);
        pros::delay(350);
        stop();

        devices.Motors.MOTOR_SCORE_ROLL.set(-60);
        pros::delay(500);
        stop();
        
        devices.Motors.MOTOR_SCORE_OPEN.set(40);
        pros::delay(150);
        stop();

        //pin now in goal, move back to get out of goal

        forward();
        pros::delay(300);
        stop();

        fourtyfivedegleft();
        pros::delay(1000);

        forward();
        pros::delay(400);
        stop();

        isAuton = false;
}

void right_side_toggle_only(){
        isAuton = true;

        toggle_auton();

        pros::delay(200);

        //turn left to score in goal
        right();
        pros::delay(580);
        stop();

        devices.Motors.MOTOR_SCORE_OPEN.set(40);
        pros::delay(350);
        stop();

        backwards();
        pros::delay(1000);
        stop();

        pros::delay(200);

        devices.Motors.MOTOR_SCORE_OPEN.set(-30);
        pros::delay(350);
        stop();

        devices.Motors.MOTOR_SCORE_ROLL.set(-60);
        pros::delay(500);
        stop();
        
        devices.Motors.MOTOR_SCORE_OPEN.set(40);
        pros::delay(150);
        stop();

        //pin now in goal, move back to get out of goal

        forward();
        pros::delay(300);
        stop();

        fourtyfivedegleft();
        pros::delay(1000);

        forward();
        pros::delay(400);
        stop();

        isAuton = false;
}

void skills_auton_routine() {
        isAuton = true;

        right_side_auton();

        fourtyfivedegright();
        pros::delay(1000);
        stop();

        forward();
        pros::delay(1000);
        stop();

        isAuton = false;
}



void nodrive() {
        isAuton = true;
        
        isAuton = false;
}

}