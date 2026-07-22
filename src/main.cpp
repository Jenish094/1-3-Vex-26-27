#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/chassis/odom.hpp"
#include "auto.h"
#include "robodash/views/selector.hpp"
#include "robotconfig.h" 
#include "robodash/api.h" // IWYU pragma: keep
#include <string>
#include <vector>
#include "controls.h"

// controls screen
static rd_view_t *controlsView = rd_view_create("Controls");
static lv_obj_t *controlsTitle = nullptr;
static lv_obj_t *controlsMoveLabel = nullptr;
static lv_obj_t *controlsLiftLabel = nullptr;
static lv_obj_t *controlsClawOpenCloseLabel = nullptr;
static lv_obj_t *controlsIntakeLabel = nullptr;
static lv_obj_t *controlsSchemeLabel = nullptr;

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
void refreshControlsPage() {
    if (currentScheme == ControlScheme::R2L2) {
        lv_label_set_text(controlsSchemeLabel, "Current: R2L2");
        lv_label_set_text(controlsMoveLabel, "Drive: LStick + R2/L2");
        lv_label_set_text(controlsLiftLabel, "Lift: Right Stick");
        lv_label_set_text(controlsClawOpenCloseLabel, "Open/Close: A");
        lv_label_set_text(controlsIntakeLabel, "Intake: R1/L1");
    }
    else {
        lv_label_set_text(controlsSchemeLabel, "Current: DSA");
        lv_label_set_text(controlsMoveLabel, "Drive: Double Stick Arcade");
        lv_label_set_text(controlsLiftLabel, "Lift: R2/L2");
        lv_label_set_text(controlsClawOpenCloseLabel, "Open/Close: A");
        lv_label_set_text(controlsIntakeLabel, "Intake: R1/L1");
    }
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

    chassis.setPose(0, 0, 0);

    lv_obj_t *controlsRoot = rd_view_obj(controlsView);
    lv_obj_set_style_pad_all(controlsRoot, 12, 0);

    controlsTitle = lv_label_create(controlsRoot);
    lv_label_set_text(controlsTitle, "Controls");
    lv_obj_align(controlsTitle, LV_ALIGN_TOP_MID, 0, 0);
    
    controlsSchemeLabel = lv_label_create(controlsRoot);
    lv_obj_align(controlsSchemeLabel, LV_ALIGN_TOP_MID, 0, 20);

    controlsMoveLabel = lv_label_create(controlsRoot);
    lv_obj_align(controlsMoveLabel, LV_ALIGN_TOP_LEFT, 0, 30);

    controlsLiftLabel = lv_label_create(controlsRoot);
    lv_label_set_text(controlsLiftLabel, "Right Stick Y: Lift");
    lv_obj_align(controlsLiftLabel, LV_ALIGN_TOP_LEFT, 0, 55);

    controlsClawOpenCloseLabel = lv_label_create(controlsRoot);
    lv_obj_align(controlsClawOpenCloseLabel, LV_ALIGN_TOP_LEFT, 0, 105);

    controlsIntakeLabel = lv_label_create(controlsRoot);
    lv_obj_align(controlsIntakeLabel, LV_ALIGN_TOP_LEFT, 0, 130);

    controlsInit();
    refreshControlsPage();
    autonSelector.focus();
} //initialize L bozo american spelling

void disabled() {
	// reset motors sensors and everything between autonomous and driver
    while (pros::competition::is_disabled()) {
        if (controlsUpdate()) {
            refreshControlsPage();
        }
        pros::delay(20);
    }
}

void competition_initialize() {
    while (pros::competition::is_disabled()) {
        if (controlsUpdate()) {
            refreshControlsPage();
        }
        pros::delay(20);
    }
}

void autonomous() {
    auto selectedAuton = autonSelector.get_auton();
    if (!selectedAuton) {
        return;
    }

    selectedAuton->action();
}

void opcontrol() {
    while (true) {
        if (controlsUpdate()) {
            refreshControlsPage();
        }
        runDriverControls();
        pros::delay(20);
    }
}
