#include "controls.h"

static bool scoreopenReverseState = false;

static void updateControllerScreen() {
    pros::Controller& controller = *devices.PRIMARY_CONTROLLER;
    controller.clear_line(0);
    controller.clear_line(1);
    controller.clear_line(2);

    controller.set_text(0, 0, "Drive: LS throttle");
    controller.set_text(1, 0, "Intake/Cascade/Score");
    controller.set_text(2, 0, "A scoreopen, B pneumatic");
}

void controlsInit() {
    devices.Motors.MOTOR_INTAKE_11W.motor->set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    devices.Motors.MOTOR_CASCADE_11WR.motor->set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    devices.Motors.MOTOR_CASCADE_11WL.motor->set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    devices.Motors.MOTOR_SCORE_OPEN.motor->set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    devices.Motors.MOTOR_SCORE_ROLL.motor->set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    devices.MGroups.DT_L.set(0);
    devices.MGroups.DT_R.set(0);
    updateControllerScreen();
}

bool controlsUpdate() {
    return false;
}
    
void runDriverControls() {
    static bool previousA = false;
    static bool previousB = false;
    pros::Controller& controller = *devices.PRIMARY_CONTROLLER;

    int throttle = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    int steer = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

    devices.MGroups.DT_L.set(throttle + steer);
    devices.MGroups.DT_R.set(throttle - steer);

    int intakePower = (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2) ? 127 : 0)
        - (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1) ? 127 : 0);
    devices.Motors.MOTOR_INTAKE_11W.set(intakePower);

    int cascadePower = (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2) ? 127 : 0)
        - (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1) ? 127 : 0);
    devices.Motors.MOTOR_CASCADE_11WR.set(cascadePower);
    devices.Motors.MOTOR_CASCADE_11WL.set(cascadePower);

    bool a = controller.get_digital(pros::E_CONTROLLER_DIGITAL_A);
    bool b = controller.get_digital(pros::E_CONTROLLER_DIGITAL_B);

    if (a && !previousA) {
        const int scoreopenDegrees = 1000;
        if (scoreopenReverseState) {
            devices.Motors.MOTOR_SCORE_OPEN.motor->move_relative(-scoreopenDegrees, 127);
        } else {
            devices.Motors.MOTOR_SCORE_OPEN.motor->move_relative(scoreopenDegrees, 127);
        }
        scoreopenReverseState = !scoreopenReverseState;
        updateControllerScreen();
    }
    
    if (b && !previousB) {
        if (devices.Pneumatics.PNEUMATIC) {
            if (devices.Pneumatics.PNEUMATIC->is_extended()) {
                devices.Pneumatics.PNEUMATIC->retract();
            } else {
                devices.Pneumatics.PNEUMATIC->extend();
            }
        }
    }

    int scoreOpenPower = (controller.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT) ? 127 : 0)
        - (controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN) ? 127 : 0);
    devices.Motors.MOTOR_SCORE_OPEN.set(scoreOpenPower);
    previousA = a;
    previousB = b;

    int scorerollPower = intakePower;
    devices.Motors.MOTOR_SCORE_ROLL.set(scorerollPower);
}
