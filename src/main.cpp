#include "main.h"
#include "movetopoint.hpp"
#include "dash/api.h"
#include "auto.h"
#include "functions.h"
#include <iostream>

bool isAuton = false;

extern const lv_img_dsc_t Logo_41103C_Blue;
extern const lv_img_dsc_t Logo_41103C_Neutral;
extern const lv_img_dsc_t Logo_41103C_Red;

rd::Image* white = nullptr;
rd::Image* blue = nullptr;
rd::Image* red = nullptr;

rd::Selector* autonSelector = nullptr;


void initialize(void) {
        pros::delay(300);
                // pros::screen::erase();
        white = new rd::Image(&Logo_41103C_Neutral, "WhiteLogo");
        blue = new rd::Image(&Logo_41103C_Blue, "BlueLogo");
        red = new rd::Image(&Logo_41103C_Red, "RedLogo");
        devices.init();
        odomReset();  // Reset tracking wheels to 0,0
        controlsInit();
        // pros::Task odomCalcPosHandler(odomCalcPos, "odomCalcPos");
        // pros::Task brainScreenManagerHandler(brainScreenManager, "brainScreenManager");
        // pros::Task dist_updateHandler(odomUpdate, "odomUpdate");

        autonSelector = new rd::Selector("autonselect", {
        {"Toggle Auton", auton::toggle_auton},
        {"Left Side Auton", auton::left_side_auton},
        {"Right Side Auton", auton::right_side_auton},
        {"Left Side Toggle Only", auton::left_side_toggle_only},
        {"Right Side Toggle Only", auton::right_side_toggle_only},
        {"Skills Auton", auton::skills_auton_routine},
        {"No Drive", auton::nodrive},
    });
        autonSelector->focus();
}

void disabled(void) {}

void competition_initialize(void) {}

void raw_drive(int pow) {
        devices.MGroups.DT_L.set(pow);
        devices.MGroups.DT_R.set(pow);
}

void autonomous(void) {
        auto selectedAuton = autonSelector->get_auton();
        if (!selectedAuton) {
                return;
        }

        AUTON_REVERSED = false;
        selectedAuton->action();

}

void opcontrol(void) {
        isAuton = false;
        controlsInit();
        functionsInit();
        while (true) {
                runDriverControls();
                updateCascadeAutomation();
                pros::delay(20);
        }
        driverScreenManager();
}

void odomUpdate(void) { // diagnostic: distance travelled by the vertical tracking wheel, in cm
        do {
                dist = (int)(verticalPosIn.load() * IN_TO_MM / 10.0f);
                pros::delay(50);
        } while (true);
}