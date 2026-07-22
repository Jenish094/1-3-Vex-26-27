#include "controls.h"
#include "robotconfig.h"

ControlScheme currentScheme = ControlScheme::R2L2;
static void updateControllerScreen() {
    controller.clear_line(0);
    controller.clear_line(1);

    switch (currentScheme) {

        case ControlScheme::R2L2:
            controller.print(0, 0, "Scheme: R2L2");
            controller.print(1, 0, "LStick + R2/L2");
            break;

        case ControlScheme::DSA:
            controller.print(0, 0, "Scheme: DSA");
            controller.print(1, 0, "Double Arcade");
            break;
    }
}
static void toggleScheme() {
    if (currentScheme == ControlScheme::R2L2)
        currentScheme = ControlScheme::DSA;
    else
        currentScheme = ControlScheme::R2L2;

    updateControllerScreen();
}
void controlsInit() {
    updateControllerScreen();
}

bool controlsUpdate() {

    static bool previousX = false;

    bool x = controller.get_digital(pros::E_CONTROLLER_DIGITAL_X);

    bool changed = false;

    if (x && !previousX) {
        toggleScheme();
        changed = true;
    }

    previousX = x;

    return changed;
}

void runDriverControls() {
    static bool clawOpenCloseState = false;
    static bool previousA = false;

    if (currentScheme == ControlScheme::R2L2) {

        int steer =
            (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2) ? 127 : 0)
          - (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2) ? 127 : 0);

        int throttle =
            controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);

        chassis.arcade(steer, throttle, false, 0.25);

        Lift.move(
            controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y));

    }
    else {

        int steer =
            controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);

        int throttle =
            controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        chassis.arcade(steer, throttle, false, 0.25);

        Lift.move(
            (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2) ? 127 : 0)
          - (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2) ? 127 : 0));
    }

    bool a = controller.get_digital(pros::E_CONTROLLER_DIGITAL_A);
    if (a && !previousA) {
        clawOpenCloseState = !clawOpenCloseState;
    }
    previousA = a;

    ClawOpenClose.set_value(clawOpenCloseState);
    Intake.move(
        (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1) ? 127 : 0)
      - (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1) ? 127 : 0)
    );
}


