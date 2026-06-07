#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "auto.h"
#include "robodash/views/selector.hpp"
#include "robotconfig.h" 
#include "robodash/api.h" // IWYU pragma: keep
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>


// location screen
static rd_view_t *locationView = rd_view_create("Location");
static lv_obj_t *locationTitle = nullptr;
static lv_obj_t *locationXLabel = nullptr;
static lv_obj_t *locationYLabel = nullptr;
static lv_obj_t *locationThetaLabel = nullptr;
static lv_obj_t *calibrateButton = nullptr;

static void update_location_labels() {
    const auto pose = chassis.getPose();

    std::ostringstream xStream;
    std::ostringstream yStream;
    std::ostringstream thetaStream;

    xStream << std::fixed << std::setprecision(2) << pose.x;
    yStream << std::fixed << std::setprecision(2) << pose.y;
    thetaStream << std::fixed << std::setprecision(2) << pose.theta;

    lv_label_set_text(locationXLabel, ("X: " + xStream.str()).c_str());
    lv_label_set_text(locationYLabel, ("Y: " + yStream.str()).c_str());
    lv_label_set_text(locationThetaLabel, ("Theta: " + thetaStream.str()).c_str());
}

static void calibrate_location_cb(lv_event_t *event) {
    if (lv_event_get_code(event) != LV_EVENT_CLICKED) {
        return;
    }

    chassis.resetLocalPosition();
    update_location_labels();
}

//init images in lvgl
extern const lv_img_dsc_t logonat;
extern const lv_img_dsc_t logonatblue;
extern const lv_img_dsc_t logonatred;

// load images into brain from C array
rd::Image white(&logonat, "WhiteLogo");
rd::Image blue(&logonatblue, "BlueLogo");
rd::Image red(&logonatred, "RedLogo");

void location_view() {
    rd_view_focus(locationView);
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

    chassis.setPose(0, 0, 0);

    lv_obj_t *locationRoot = rd_view_obj(locationView);
    lv_obj_set_style_pad_all(locationRoot, 12, 0);

    locationTitle = lv_label_create(locationRoot);
    lv_label_set_text(locationTitle, "Location");
    lv_obj_align(locationTitle, LV_ALIGN_TOP_MID, 0, 0);

    locationXLabel = lv_label_create(locationRoot);
    lv_label_set_text(locationXLabel, "X: 0.00");
    lv_obj_align(locationXLabel, LV_ALIGN_TOP_LEFT, 0, 30);

    locationYLabel = lv_label_create(locationRoot);
    lv_label_set_text(locationYLabel, "Y: 0.00");
    lv_obj_align(locationYLabel, LV_ALIGN_TOP_LEFT, 0, 55);

    locationThetaLabel = lv_label_create(locationRoot);
    lv_label_set_text(locationThetaLabel, "Theta: 0.00");
    lv_obj_align(locationThetaLabel, LV_ALIGN_TOP_LEFT, 0, 80);

    calibrateButton = lv_btn_create(locationRoot);
    lv_obj_set_size(calibrateButton, 120, 36);
    lv_obj_align(calibrateButton, LV_ALIGN_TOP_LEFT, 0, 115);
    lv_obj_add_event_cb(calibrateButton, calibrate_location_cb, LV_EVENT_CLICKED, nullptr);

    lv_obj_t *calibrateLabel = lv_label_create(calibrateButton);
    lv_label_set_text(calibrateLabel, "Calibrate");
    lv_obj_center(calibrateLabel);

    update_location_labels();

    autonSelector.focus();

    pros::Task locationTask([]() {
        while (true) {
            update_location_labels();
            pros::delay(50);
        }
    });
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

        static bool claw_state = false;
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
            claw_state = !claw_state;
            Claw.set_value(claw_state);
        }
        pros::delay(20);                               // Run for 20 ms then update
	}
}

