#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/chassis/odom.hpp"
#include "auto.h"
#include "robodash/views/selector.hpp"
#include "robotconfig.h" 
#include "robodash/api.h" // IWYU pragma: keep
#include <string>
#include <vector>

// controls screen
static rd_view_t *controlsView = rd_view_create("Controls");
static lv_obj_t *controlsTitle = nullptr;
static lv_obj_t *controlsMoveLabel = nullptr;
static lv_obj_t *controlsLiftLabel = nullptr;
static lv_obj_t *controlsClawLabel = nullptr;
static lv_obj_t *controlsClawOpenCloseLabel = nullptr;

//init images in lvgl
extern const lv_img_dsc_t logonat;
extern const lv_img_dsc_t logonatblue;
extern const lv_img_dsc_t logonatred;

// load images into brain from C array
rd::Image white(&logonat, "WhiteLogo");
rd::Image blue(&logonatblue, "BlueLogo");
rd::Image red(&logonatred, "RedLogo");

void controls_view() {
    rd_view_focus(controlsView);
}

// setup autonomous selector with custom lib
rd::Selector autonSelector("autonselect", {
    {"BlueLeft", blueFront},
    {"BlueRight", blueLeft},
    {"RedLeft", redFront},
    {"RedRight", redLeft},
    {"Skills", skills_auton},
});


void initialize() {
    chassis.calibrate();
    lemlib::init();
    imu.reset();

    chassis.setPose(0, 0, 0);

    lv_obj_t *controlsRoot = rd_view_obj(controlsView);
    lv_obj_set_style_pad_all(controlsRoot, 12, 0);

    controlsTitle = lv_label_create(controlsRoot);
    lv_label_set_text(controlsTitle, "Controls");
    lv_obj_align(controlsTitle, LV_ALIGN_TOP_MID, 0, 0);

    controlsMoveLabel = lv_label_create(controlsRoot);
    lv_label_set_text(controlsMoveLabel, "L2/R2: Drive");
    lv_obj_align(controlsMoveLabel, LV_ALIGN_TOP_LEFT, 0, 30);

    controlsLiftLabel = lv_label_create(controlsRoot);
    lv_label_set_text(controlsLiftLabel, "Right Stick Y: Lift");
    lv_obj_align(controlsLiftLabel, LV_ALIGN_TOP_LEFT, 0, 55);

    controlsClawLabel = lv_label_create(controlsRoot);
    lv_label_set_text(controlsClawLabel, "A: Claw Rotate");
    lv_obj_align(controlsClawLabel, LV_ALIGN_TOP_LEFT, 0, 80);

    controlsClawOpenCloseLabel = lv_label_create(controlsRoot);
    lv_label_set_text(controlsClawOpenCloseLabel, "B: Claw Open/Close");
    lv_obj_align(controlsClawOpenCloseLabel, LV_ALIGN_TOP_LEFT, 0, 105);

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
        const int throttle = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X); // ts is actually steer not throttle
        const int steer = (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2) ? 127 : 0)
                        - (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2) ? 127 : 0); // this is throttle not steer, the code doesnt like it when i make it normal
        chassis.arcade(throttle, steer, false, 0.25);
        Lift.move(pros::E_CONTROLLER_ANALOG_RIGHT_Y);
        ClawOpenClose.set_value(controller.get_digital(pros::E_CONTROLLER_DIGITAL_B));
        ClawRotate.set_value(controller.get_digital(pros::E_CONTROLLER_DIGITAL_A));
        pros::delay(20);                               // Run for 20 ms then update
	}
}

