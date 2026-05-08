#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "robotconfig.h" 
#include "robodash/api.h" // IWYU pragma: keep

extern const lv_img_dsc_t logonat;
extern const lv_img_dsc_t logonatblue;
extern const lv_img_dsc_t logonatred;

rd::Image white(&logonat, "WhiteLogo");
rd::Image blue(&logonatblue, "BlueLogo");
rd::Image red(&logonatred, "RedLogo");


//Autonomous routines
void redleft(){

}
void redright(){

}
void blueleft(){

}
void blueright(){
	
}



void skills_auton(){

}

rd::Selector selector({
    {"RedLeft", redleft},
    {"RedRight", redright},
    {"BlueLeft", blueleft},
	{"BlueRight", blueright},
	{"Skills", skills_auton}
});


void initialize() {
    // pros::lcd::initialize(); // initialize brain screen
    // while (true) { // infinite loop
    //     // print measurements from the adi encoder
    //     pros::lcd::print(0, "Vertical Encoder: %i", verticalEnc.get_value());
    //     // print measurements from the rotation sensor
    //     pros::lcd::print(1, "Horizontal Sensor: %i", horizontalEnc.get_value());
    //     pros::delay(10); // delay to save resources. DO NOT REMOVE
// } // temporary llemu code to determine tracking wheel reversal.

} //initialize L bozo american spelling

void disabled() {
	// reset motors sensors and everything between autonomous and driver
}

void competition_initialize() {}

void autonomous() {
// selector.run_auton();
    // set position to x:0, y:0, heading:0
    chassis.setPose(0, 0, 0);
    // turn to face heading 90 with a very long timeout
    chassis.turnToHeading(90, 100000);
}

void opcontrol() {
	while (true) {
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        chassis.curvature(rightX, leftY);
        pros::delay(20);                               // Run for 20 ms then update
	}
}
