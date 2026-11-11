#include "main.h"
#include "movetopoint.hpp"
#include "dash/api.h"
#include "auto.h"
#include "functions.h"
#include <iostream>

bool isAuton = false;
bool allianceIsRed = false; // false = blue, true = red

extern const lv_img_dsc_t Logo_41103C_Blue;
extern const lv_img_dsc_t Logo_41103C_Neutral;
extern const lv_img_dsc_t Logo_41103C_Red;
extern const lv_img_dsc_t Logo_41103C_Chinese;

rd::Image* white = nullptr;
rd::Image* blue = nullptr;
rd::Image* red = nullptr;
rd::Image* chinese = nullptr;
rd::Selector* autonSelector = nullptr;


void initialize(void) {
        pros::delay(300);
                // pros::screen::erase();
        white = new rd::Image(&Logo_41103C_Neutral, "WhiteLogo");
        blue = new rd::Image(&Logo_41103C_Blue, "BlueLogo");
        red = new rd::Image(&Logo_41103C_Red, "RedLogo");
        chinese = new rd::Image(&Logo_41103C_Chinese, "ChineseLogo");
        devices.init();
        initializeRecorderTask();
        odomReset();  // Reset tracking wheels to 0,0
        controlsInit();

        autonSelector = new rd::Selector("autonselect", {
        {"1 Pin Left", auton::one_pin_left},
        {"1 Pin Right", auton::one_pin_right},
        // {"Middle Pin Left", auton::middle_pin_left},
        // {"Middle Pin Right", auton::middle_pin_right},
        {"30 Point Left", auton::thirty_point_left},
        {"30 Point Right", auton::thirty_point_right},
        {"40 Point Left", auton::forty_point_left},
        {"40 Point Right", auton::forty_point_right},
        {"Skills Routine", auton::skills_routine},
        {"Recorded Routine", auton::recorded_routine},
        {"No Drive", auton::nodrive},
    });
        autonSelector->focus();
}

void disabled(void) {
        recorder::stop();
}

void competition_initialize(void) {}

void autonomous(void) {
        recorder::stop();
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
}