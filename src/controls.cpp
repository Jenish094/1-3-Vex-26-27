#include "controls.h"
#include <cmath>
#include <cstdint>

extern bool allianceIsRed;

namespace {
constexpr std::uint32_t scorerollFinishTimeMs = 500;
}

void controlsInit() {
    devices.Motors.MOTOR_INTAKE_11W.motor->set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    devices.Motors.MOTOR_CASCADE_11WR.motor->set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    devices.Motors.MOTOR_CASCADE_11WL.motor->set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    devices.Motors.MOTOR_SCORE_OPEN.motor->set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    devices.Motors.MOTOR_SCORE_ROLL.motor->set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    devices.MGroups.DT_L.set(0);
    devices.MGroups.DT_R.set(0);
}

bool controlsUpdate() {
    return false;
}
    
void runDriverControls() {
    static bool previousA = false;
    static bool previousB = false;
    static bool seekingColor = false;
    static bool finishingColor = false;
    static std::uint32_t colorSeenTime = 0;
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
        seekingColor = !seekingColor;
        finishingColor = false;
        if (seekingColor) {
            colorSeenTime = 0;
        }
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

    int scorerollPower = intakePower;
    if (seekingColor) {
        scorerollPower = -127;

        if (!finishingColor && devices.Sensors.ToggleOptical) {
            const double hue = devices.Sensors.ToggleOptical->get_hue();
            const int proximity = devices.Sensors.ToggleOptical->get_proximity();
            const bool validReading = std::isfinite(hue) && hue >= 0.0 && proximity >= 100;
            const bool isRed = validReading && (hue <= 30.0 || hue >= 330.0);
            const bool isBlue = validReading && hue >= 190.0 && hue <= 260.0;
            const bool targetSeen = allianceIsRed ? isRed : isBlue;

            if (targetSeen) {
                finishingColor = true;
                colorSeenTime = pros::millis();
            }
        }

        if (finishingColor &&
            pros::millis() - colorSeenTime >= scorerollFinishTimeMs) {
            seekingColor = false;
            finishingColor = false;
            scorerollPower = 0;
        }
    }
    previousA = a;
    previousB = b;

    devices.Motors.MOTOR_SCORE_ROLL.set(scorerollPower);

}
