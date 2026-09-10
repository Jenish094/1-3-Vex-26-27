#include "main.h"
#include "movetopoint.hpp"
#include "robodash/api.h"
#include "auto.h"
#include "functions.h"
#include <iostream>

bool isAuton = false;

extern const lv_img_dsc_t logonat;
extern const lv_img_dsc_t logonatblue;
extern const lv_img_dsc_t logonatred;

rd::Image white(&logonat, "WhiteLogo");
rd::Image blue(&logonatblue, "BlueLogo");
rd::Image red(&logonatred, "RedLogo");

rd::Selector autonSelector("autonselect", {
        {"CoenAuton_LeftSide", CoenAuton_LeftSide},
        {"CoenAuton_RightSide", CoenAuton_RightSide},
        {"ToggleOnly_LeftSide", ToggleOnly_LeftSide},
        {"ToggleOnly_RightSide", ToggleOnly_RightSide},
        {"Skills", skills_auton},
        {"nodrive", nodrive},
    //{"ToggleOnly", runToggleNew},
    //{"PinPickUp", pinpickup},
    //{"BluegoalsLeftNew", bluegoalsLeftNew},
    //{"BluegoalsRight", bluegoalsRight},
    //{"RedgoalsLeft", redgoalsLeft},
    //{"RedgoalsRight", redgoalsRight},
    
});

void initialize(void) {
        pros::delay(300);
        pros::screen::erase();
        devices.init();
        odomReset();  // Reset tracking wheels to 0,0
        controlsInit();
        // pros::Task odomCalcPosHandler(odomCalcPos, "odomCalcPos");
        // pros::Task brainScreenManagerHandler(brainScreenManager, "brainScreenManager");
        // pros::Task dist_updateHandler(odomUpdate, "odomUpdate");

        autonSelector.focus();
}

void disabled(void) {}

void competition_initialize(void) {}

void raw_drive(int pow) {
        devices.MGroups.DT_L.set(pow);
        devices.MGroups.DT_R.set(pow);
}

void autonomous(void) {
//     auto selectedAuton = autonSelector.get_auton();
//     if (!selectedAuton) {
//         return; 
//     }

//     selectedAuton->action();
        moveToPoint(12.5, 8.3);}

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