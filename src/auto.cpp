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

void forward() {
        devices.MGroups.DT_L.set(60);
        devices.MGroups.DT_R.set(60);
}

void backwards() {
        devices.MGroups.DT_L.set(-60);
        devices.MGroups.DT_R.set(-60);
}

void stop() {
        devices.MGroups.DT_L.set(0);
        devices.MGroups.DT_R.set(0);
}

void toggle_auton(){
        isAuton = true;

        


        devices.MGroups.DT_L.set(-60);
        devices.MGroups.DT_R.set(-60);
        pros::delay(350);
        devices.MGroups.DT_L.set(0);
        devices.MGroups.DT_R.set(0);

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
        devices.MGroups.DT_L.set(-60);
        devices.MGroups.DT_R.set(-60);
        pros::delay(600);
        devices.MGroups.DT_L.set(0);
        devices.MGroups.DT_R.set(0);

        // Move robot back at double speed
        devices.MGroups.DT_L.set(60);
        devices.MGroups.DT_R.set(60);
        pros::delay(600);
        devices.MGroups.DT_L.set(0);
        devices.MGroups.DT_R.set(0);

        // Move robot slightly forward
        devices.MGroups.DT_L.set(60);
        devices.MGroups.DT_R.set(60);
        pros::delay(100);
        devices.MGroups.DT_L.set(0);
        devices.MGroups.DT_R.set(0);

        isAuton = false;
}

void left_side_auton(){
        isAuton = true;

        toggle_auton();

        pros::delay(200);

        //turn left to score in goal
        devices.MGroups.DT_L.set(-60);
        devices.MGroups.DT_R.set(60);
        pros::delay(580);
        stop();

        devices.Motors.MOTOR_SCORE_OPEN.set(40);
        pros::delay(350);
        stop();

        backwards();
        pros::delay(900);
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

        devices.MGroups.DT_L.set(60);
        devices.MGroups.DT_R.set(-60);
        pros::delay(510);
        stop();

        devices.Motors.MOTOR_SCORE_ROLL.set(60);
        pros::delay(100);
        
        backwards();
        pros::delay(1110);
        stop();

        devices.Motors.MOTOR_SCORE_OPEN.set(-30);
        pros::delay(400);
        stop();

        devices.Motors.MOTOR_SCORE_OPEN.set(30);
        pros::delay(200);
        stop();

        forward();
        pros::delay(500);
        stop();

        fourtyfivedegleft();
        pros::delay(1000);

        backwards();
        pros::delay(1000);
        stop();

        devices.Motors.MOTOR_SCORE_OPEN.set(-30);
        pros::delay(400);
        stop();

        devices.Motors.MOTOR_SCORE_ROLL.set(-60);
        pros::delay(100);



        isAuton = false;
}

void right_side_auton(){
        isAuton = true;
        isAuton = false;
}

void left_side_toggle_only(){
        isAuton = true;
        isAuton = false;
}

void right_side_toggle_only(){
        isAuton = true;
        isAuton = false;
}

void skills_auton_routine() {
        isAuton = true;

        isAuton = false;
}


void nodrive() {
        isAuton = true;
        isAuton = false;
}

}
