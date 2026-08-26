#include "main.h"

// my headers
// #include "robotconfig.h"

// External API
// #include "robodash/api.h" // IWYU pragma: keep
#include "lemlib/api.hpp" // IWYU pragma: keep

pros::MotorGroup leftdt({-14, -15});
pros::MotorGroup rightdt({11, 12});

lemlib::Drivetrain drivetrain(&leftdt,&rightdt, 14.9606, 
                lemlib::Omniwheel::NEW_275, 450, 0);

//Odometry
pros::Imu imu(5);
pros::Rotation horizontalEnc(4);
pros::Rotation verticalEnc(3);

void initialize() {

    pros::lcd::initialize(); // initialize brain screen
    while (true) { // infinite loop
        // print measurements from the adi encoder
        pros::lcd::print(0, "Horiz Sensor: %i", horizontalEnc.get_position());
        // print measurements from the rotation sensor
        pros::lcd::print(1, "Vertical Sensor: %i", verticalEnc.get_position());
        pros::delay(10); // delay to save resources. DO NOT REMOVE
    }

}

void disabled() {

}

void competition_initialize() {

}

void autonomous() {

}

void opcontrol() {

}

