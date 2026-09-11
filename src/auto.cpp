#include "hardware.hpp"
#include "main.h"
#include "movetopoint.hpp"
#include "odom.hpp"
#include "auto.h"
#include <cmath>
#include "api.h"

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

void runToggleNew(){
        isAuton = true;
        forward();
        pros::delay(550);
        stop();
        
        //backwards();
        //pros::delay(600);
        //stop();


        // reset toggle mech
        devices.Motors.MOTOR_SCORE_OPEN.set(-60);
        pros::delay(20);
        devices.Motors.MOTOR_SCORE_OPEN.set(0);
        devices.Motors.MOTOR_SCORE_OPEN.set(60);
        pros::delay(520);
        devices.Motors.MOTOR_SCORE_OPEN.set(0);

        // back into the toggle
        backwards();
        pros::delay(500);
        stop();

        // flip toggle once
        devices.Motors.MOTOR_SCORE_OPEN.set(60);
        pros::delay(100);
        devices.Motors.MOTOR_SCORE_OPEN.set(0);

        devices.Motors.MOTOR_CASCADE_11WL.set(60);
        devices.Motors.MOTOR_CASCADE_11WR.set(60);
        pros::delay(1500);
        devices.Motors.MOTOR_CASCADE_11WL.set(0);
        devices.Motors.MOTOR_CASCADE_11WR.set(0);

        backwards();
        pros::delay(110);
        stop();


        //move up again
        forward();
        pros::delay(250);
        stop();

        devices.Motors.MOTOR_CASCADE_11WL.set(-60);
        devices.Motors.MOTOR_CASCADE_11WR.set(-60);
        pros::delay(1600);
        devices.Motors.MOTOR_CASCADE_11WL.set(0);
        devices.Motors.MOTOR_CASCADE_11WR.set(0);


        // reset toggle mech
        devices.Motors.MOTOR_SCORE_OPEN.set(-60);
        pros::delay(600);
        devices.Motors.MOTOR_SCORE_OPEN.set(0);
        devices.Motors.MOTOR_SCORE_OPEN.set(60);
        pros::delay(520);
        devices.Motors.MOTOR_SCORE_OPEN.set(0);


        // back into toggle again
        backwards();
        pros::delay(450);
        stop();

        // flip the toggle second time
        devices.Motors.MOTOR_SCORE_OPEN.set(60);
        pros::delay(200);
        devices.Motors.MOTOR_SCORE_OPEN.set(0);

        devices.Motors.MOTOR_CASCADE_11WL.set(60);
        devices.Motors.MOTOR_CASCADE_11WR.set(60);
        pros::delay(1500);
        devices.Motors.MOTOR_CASCADE_11WL.set(0);
        devices.Motors.MOTOR_CASCADE_11WR.set(0);

        backwards();
        pros::delay(170);
        stop();
        
        forward();
        pros::delay(400);
        stop();

        devices.Motors.MOTOR_CASCADE_11WL.set(-60);
        devices.Motors.MOTOR_CASCADE_11WR.set(-60);
        pros::delay(1600);
        devices.Motors.MOTOR_CASCADE_11WL.set(0);
        devices.Motors.MOTOR_CASCADE_11WR.set(0);
        isAuton = false;
}


void bluegoalsLeftNew() {
        isAuton = true;
        odomReset();

        // starting angled to the goal need to check distance
        devices.MGroups.DT_L.set(20);
        devices.MGroups.DT_R.set(20);
        pros::delay(1000);
        stop();
        //Now at goal dropped pin

        pros::delay(200); //pause to allow pin to be seated

        // move slightly back need to chekc if this is enough
        backwards();
        pros::delay(150);
        stop();

        // turn 45 deg left to be back parallel to the toggle
        fourtyfivedegright();
        
        // move backwards near the toggle need to check if this is enough
        forward();
        pros::delay(790);
        stop();

        // turn 45 left to be back facing the toggle
        devices.MGroups.DT_L.set(-60);
        devices.MGroups.DT_R.set(60);
        pros::delay(280);
        devices.MGroups.DT_L.set(0);
        devices.MGroups.DT_R.set(0);

        // smack into wall for alignment
        backwards();
        pros::delay(1000);
        stop();

        
        runToggleNew();

        isAuton = false;

}

void pinpickup() {
        isAuton = true;

        devices.Motors.MOTOR_SCORE_OPEN.set(-45); //move down to take in pin
        pros::delay(1200);
        stop();

        devices.Motors.MOTOR_SCORE_OPEN.set(30); //lift back up so we can move around
        pros::delay(1200);
        stop();

        devices.Motors.MOTOR_SCORE_ROLL.set(40); //stop intaking 
        pros::delay(10);
        stop();

        isAuton = false;

}

void LeftAuto() {
        isAuton = true;
        odomReset();
        runToggleNew();

        //Move the pin in tight in the scoreflipper
        devices.Motors.MOTOR_SCORE_ROLL.set(60);
        pros::delay(60);
        devices.Motors.MOTOR_SCORE_ROLL.set(0);

        devices.MGroups.DT_L.set(-60);
        devices.MGroups.DT_R.set(-60);
        pros::delay(100);
        devices.MGroups.DT_L.set(0);
        devices.MGroups.DT_R.set(0);


        //Turn right enough
        devices.MGroups.DT_L.set(-60);
        devices.MGroups.DT_R.set(+60);
        pros::delay(500);
        devices.MGroups.DT_L.set(0);
        devices.MGroups.DT_R.set(0);

        // set flipper to zero
        devices.Motors.MOTOR_SCORE_OPEN.set(-60);
        pros::delay(10);
        devices.Motors.MOTOR_SCORE_OPEN.set(0);

        //up the flipper
        devices.Motors.MOTOR_SCORE_OPEN.set(60);
        pros::delay(450);
        devices.Motors.MOTOR_SCORE_OPEN.set(0);

        // Move backwards
        devices.MGroups.DT_L.set(-60);
        devices.MGroups.DT_R.set(-60);
        pros::delay(1120);
        devices.MGroups.DT_L.set(0);
        devices.MGroups.DT_R.set(0);

        //Drop the pin
        devices.Motors.MOTOR_SCORE_ROLL.set(-60);
        pros::delay(500);
        devices.Motors.MOTOR_SCORE_ROLL.set(0);

        isAuton = false;
}

void RightAuto() {
        isAuton = true;
        odomReset();
        runToggleNew();

        //Move the pin in tight in the scoreflipper
        devices.Motors.MOTOR_SCORE_ROLL.set(60);
        pros::delay(60);
        devices.Motors.MOTOR_SCORE_ROLL.set(0);

        devices.MGroups.DT_L.set(-60);
        devices.MGroups.DT_R.set(-60);
        pros::delay(100);
        devices.MGroups.DT_L.set(0);
        devices.MGroups.DT_R.set(0);


        //Turn right enough
        devices.MGroups.DT_L.set(60);
        devices.MGroups.DT_R.set(-60);
        pros::delay(500);
        devices.MGroups.DT_L.set(0);
        devices.MGroups.DT_R.set(0);

        // set flipper to zero
        devices.Motors.MOTOR_SCORE_OPEN.set(-60);
        pros::delay(10);
        devices.Motors.MOTOR_SCORE_OPEN.set(0);

        //up the flipper
        devices.Motors.MOTOR_SCORE_OPEN.set(60);
        pros::delay(450);
        devices.Motors.MOTOR_SCORE_OPEN.set(0);

        // Move backwards
        devices.MGroups.DT_L.set(-60);
        devices.MGroups.DT_R.set(-60);
        pros::delay(1120);
        devices.MGroups.DT_L.set(0);
        devices.MGroups.DT_R.set(0);

        //Drop the pin
        devices.Motors.MOTOR_SCORE_ROLL.set(-60);
        pros::delay(500);
        devices.Motors.MOTOR_SCORE_ROLL.set(0);

        isAuton = false;
}

void ToggleOnly_LeftSide() {
        isAuton = true;
        devices.MGroups.DT_L.set(35);
        devices.MGroups.DT_R.set(35);
        pros::delay(700);
        stop();

        devices.MGroups.DT_L.set(10);//Shake to ensure pin is seated
        devices.MGroups.DT_R.set(10);

        backwards();//reverse from goal
        pros::delay(100);
        stop();

        forward();
        pros::delay(100);
        stop();
        //Now at goal dropped pin

        devices.MGroups.DT_L.set(25);
        devices.MGroups.DT_R.set(25);

        pros::delay(200); 

        // move slightly back need to chekc if this is enough
        backwards();
        pros::delay(150);
        stop();

        // turn 45 deg left to be back parallel to the toggle
        fourtyfivedegright();
        
        // move backwards near the toggle need to check if this is enough
        forward();
        pros::delay(800);
        stop();

        // turn 45 left to be back facing the toggle
        devices.MGroups.DT_L.set(-60);
        devices.MGroups.DT_R.set(60);
        pros::delay(280);
        devices.MGroups.DT_L.set(0);
        devices.MGroups.DT_R.set(0);

        devices.MGroups.MOTORGROUP_INTAKE.set(127);
        pros::delay(500);
        devices.MGroups.MOTORGROUP_INTAKE.set(0);

        backwards();
        pros::delay(1200);
        stop();

        forward();
        pros::delay(600);
        stop();
        
        // turn 45 left to be back facing the toggle
        devices.MGroups.DT_L.set(-60);
        devices.MGroups.DT_R.set(60);
        pros::delay(1300);
        devices.MGroups.DT_L.set(0);
        devices.MGroups.DT_R.set(0);

        pros::delay(100);

        forward();
        pros::delay(610);
        stop();

        backwards();
        pros::delay(600);
        stop();

        forward();
        pros::delay(620);
        stop();

        backwards();
        pros::delay(600);
        stop();
        isAuton = false;
}

void ToggleOnly_RightSide() {
        isAuton = true;

        devices.MGroups.DT_L.set(35);
        devices.MGroups.DT_R.set(35);
        pros::delay(700);
        stop();

        devices.MGroups.DT_L.set(10);//Shake to ensure pin is seated
        devices.MGroups.DT_R.set(10);

        backwards();//reverse from goal
        pros::delay(100);
        stop();

        forward();
        pros::delay(100);
        stop();
        //Now at goal dropped pin

        devices.MGroups.DT_L.set(25);
        devices.MGroups.DT_R.set(25);

        pros::delay(200); 

        // move slightly back need to chekc if this is enough
        backwards();
        pros::delay(150);
        stop();

        // turn 45 deg left to be back parallel to the toggle
        fourtyfivedegleft();
        
        // move backwards near the toggle need to check if this is enough
        forward();
        pros::delay(800);
        stop();

        // turn 45 left to be back facing the toggle
        devices.MGroups.DT_L.set(60);
        devices.MGroups.DT_R.set(-60);
        pros::delay(280);
        devices.MGroups.DT_L.set(0);
        devices.MGroups.DT_R.set(0);

        devices.MGroups.MOTORGROUP_INTAKE.set(127);
        pros::delay(500);
        devices.MGroups.MOTORGROUP_INTAKE.set(0);

        backwards();
        pros::delay(1200);
        stop();

        forward();
        pros::delay(600);
        stop();
        
        // turn 45 left to be back facing the toggle
        devices.MGroups.DT_L.set(60);
        devices.MGroups.DT_R.set(-60);
        pros::delay(1300);
        devices.MGroups.DT_L.set(0);
        devices.MGroups.DT_R.set(0);

        pros::delay(100);

        forward();
        pros::delay(610);
        stop();

        backwards();
        pros::delay(600);
        stop();

        forward();
        pros::delay(620);
        stop();

        backwards();
        pros::delay(600);
        stop();
        isAuton = false;
}

void bluegoalsRight() {
        isAuton = true;
        isAuton = false;
}

// void redgoalsLeftOdom() {
//         isAuton = true;
//         odomReset();
//         auton_driveToPoint(-0.465, 20.013, 5000, false);
//         auton_driveToPoint(-16.755, 18.772, 5000, true);
// }

void redgoalsLeft() {
        isAuton = true;
        LeftAuto();
        isAuton = false;
}

void redgoalsRight() {
        isAuton = true;
        RightAuto();
        isAuton = false;
}


void skills_auton() {
        isAuton = true;
        ToggleOnly_LeftSide();
        backwards();
        pros::delay(4000);
        stop();
        isAuton = false;
}


void nodrive() {
        isAuton = true;
        isAuton = false;
}

//Coen testing if i can add stuff to the auto.cpp file and have it work in the main.cpp file