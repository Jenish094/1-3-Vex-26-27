#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "auto.h"
#include "robodash/views/selector.hpp"
#include "robotconfig.h" 
#include "robodash/api.h" // IWYU pragma: keep
#include <string>
#include <vector>

extern const lv_img_dsc_t logonat;
extern const lv_img_dsc_t logonatblue;
extern const lv_img_dsc_t logonatred;

rd::Image white(&logonat, "WhiteLogo");
rd::Image blue(&logonatblue, "BlueLogo");
rd::Image red(&logonatred, "RedLogo");
rd::Console location("Location");

void location_view() {
    location.focus();
}

rd::Selector autonSelector("autonselect", {
    {"Nothing", nothing_auton},
    {"BlueLeft", blueleft},
    {"BlueRight", blueright},
    {"RedLeft", redleft},
    {"RedRight", redright},
    {"Skills", skills_auton},
});


void initialize() {
    chassis.calibrate();
    autonSelector.focus();
} //initialize L bozo american spelling

void disabled() {
	// reset motors sensors and everything between autonomous and driver
}

void competition_initialize() {}

void autonomous() {
    auto selectedAuton = autonSelector.get_auton();
    if (!selectedAuton) {
        return;
    }

    selectedAuton->action();
}

void opcontrol() {
	while (true) {
        const int throttle = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);
        const int steer = (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2) ? 127 : 0)
                        - (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2) ? 127 : 0);
        chassis.arcade(throttle, steer);
        pros::delay(20);                               // Run for 20 ms then update
	}
}

