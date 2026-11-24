#include "controls.h"
#include <cmath>
#include <cstdint>

extern bool allianceIsRed;

namespace {
constexpr std::uint32_t scorerollFinishTimeMs = 500;

enum class PinGrabState {
    Idle,
    MovingUp,
    MovingWristDown,
    MovingBackDown,
    RollingIntake
};

constexpr float cascadeMoveUpUnits = 1800.0f;
constexpr float cascadeToleranceUnits = 200.0f;
constexpr double wristDownDegrees = -90.0;
constexpr int scoreRollMacroPower = -127;
constexpr std::uint32_t scoreRollMacroTimeMs = 500;
constexpr std::uint32_t cascadeTimeoutMs = 5000;
constexpr std::uint32_t ratchetWristMoveTimeMs = 400;
bool ratchetWristMoveDown = true;
bool ratchetWristMacroActive = false;
std::uint32_t ratchetWristMacroStart = 0;
}

void controlsInit() {
    ratchetWristMoveDown = true;
    ratchetWristMacroActive = false;
    ratchetWristMacroStart = 0;
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
    static bool previousX = false;
    static bool previousY = false;
    static bool seekingColor = false;
    static bool finishingColor = false;
    static std::uint32_t colorSeenTime = 0;
    static PinGrabState pinGrabState = PinGrabState::Idle;
    static float pinGrabCascadeStart = 0.0f;
    static float pinGrabCascadeTarget = 0.0f;
    static double pinGrabWristTarget = 0.0;
    static std::uint32_t stageStartTime = 0;
    static std::uint32_t scoreRollMacroStart = 0;
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
    if (cascadePower != 0 && pinGrabState != PinGrabState::Idle) {
        pinGrabState = PinGrabState::Idle;
        devices.Motors.MOTOR_SCORE_ROLL.set(0);
    }
    devices.Motors.MOTOR_CASCADE_11WR.set(cascadePower);
    devices.Motors.MOTOR_CASCADE_11WL.set(cascadePower);

    bool a = controller.get_digital(pros::E_CONTROLLER_DIGITAL_A);
    bool b = controller.get_digital(pros::E_CONTROLLER_DIGITAL_B);
    bool x = controller.get_digital(pros::E_CONTROLLER_DIGITAL_X);
    bool y = controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y);

    if (x && !previousX) {
        pinGrabState = PinGrabState::Idle;
        devices.Motors.MOTOR_CASCADE_11WR.set(0);
        devices.Motors.MOTOR_CASCADE_11WL.set(0);
        devices.Motors.MOTOR_SCORE_ROLL.set(0);
        if (devices.Motors.MOTOR_SCORE_OPEN.motor) {
            devices.Motors.MOTOR_SCORE_OPEN.motor->brake();
        }
    }

    if (a && !previousA && pinGrabState == PinGrabState::Idle &&
        devices.Sensors.CascadeEncoder &&
        devices.Motors.MOTOR_SCORE_OPEN.motor) {
        pinGrabCascadeStart = devices.Sensors.CascadeEncoder->get_position();
        pinGrabCascadeTarget = pinGrabCascadeStart + cascadeMoveUpUnits;
        pinGrabWristTarget =
            devices.Motors.MOTOR_SCORE_OPEN.motor->get_position() + wristDownDegrees;
        stageStartTime = pros::millis();
        pinGrabState = PinGrabState::MovingUp;
    }

    if (pinGrabState != PinGrabState::Idle &&
        devices.Sensors.CascadeEncoder &&
        devices.Motors.MOTOR_SCORE_OPEN.motor) {
        const float cascadePosition =
            devices.Sensors.CascadeEncoder->get_position();
        const double wristPosition =
            devices.Motors.MOTOR_SCORE_OPEN.motor->get_position();

        if (pinGrabState == PinGrabState::MovingUp) {
            const float error = pinGrabCascadeTarget - cascadePosition;
            if (std::fabs(error) <= cascadeToleranceUnits ||
                pros::millis() - stageStartTime >= cascadeTimeoutMs) {
                devices.Motors.MOTOR_CASCADE_11WR.set(0);
                devices.Motors.MOTOR_CASCADE_11WL.set(0);
                pinGrabState = PinGrabState::MovingWristDown;
                devices.Motors.MOTOR_SCORE_OPEN.motor->move_relative(
                    wristDownDegrees, 127);
            } else {
                const int power = error > 0.0f ? 127 : -127;
                devices.Motors.MOTOR_CASCADE_11WR.set(power);
                devices.Motors.MOTOR_CASCADE_11WL.set(power);
            }
        } else if (pinGrabState == PinGrabState::MovingWristDown) {
            if (std::fabs(wristPosition - pinGrabWristTarget) <= 5.0 ||
                pros::millis() - stageStartTime >= cascadeTimeoutMs) {
                stageStartTime = pros::millis();
                pinGrabState = PinGrabState::MovingBackDown;
            }
        } else if (pinGrabState == PinGrabState::MovingBackDown) {
            const float error = pinGrabCascadeStart - cascadePosition;
            if (std::fabs(error) <= cascadeToleranceUnits ||
                pros::millis() - stageStartTime >= cascadeTimeoutMs) {
                devices.Motors.MOTOR_CASCADE_11WR.set(0);
                devices.Motors.MOTOR_CASCADE_11WL.set(0);
                scoreRollMacroStart = pros::millis();
                pinGrabState = PinGrabState::RollingIntake;
            } else {
                const int power = error > 0.0f ? 127 : -127;
                devices.Motors.MOTOR_CASCADE_11WR.set(power);
                devices.Motors.MOTOR_CASCADE_11WL.set(power);
            }
        } else if (pinGrabState == PinGrabState::RollingIntake) {
            if (pros::millis() - scoreRollMacroStart >= scoreRollMacroTimeMs) {
                pinGrabState = PinGrabState::Idle;
                devices.Motors.MOTOR_SCORE_ROLL.set(0);
            } else {
                devices.Motors.MOTOR_SCORE_ROLL.set(scoreRollMacroPower);
            }
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

    if (y && !previousY) {
        ratchetWristMacroActive = true;
        ratchetWristMacroStart = pros::millis();
    }

    int ratchetWristPower = 0;
    if (ratchetWristMacroActive) {
        if (pros::millis() - ratchetWristMacroStart >= ratchetWristMoveTimeMs) {
            ratchetWristMacroActive = false;
        } else {
            ratchetWristPower = ratchetWristMoveDown ? -127 : 127;
        }
    }

    if (y && !previousY) {
        ratchetWristMoveDown = !ratchetWristMoveDown;
    }

    int scoreOpenPower = (controller.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT) ? 127 : 0)
        - (controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN) ? 127 : 0);
    if (ratchetWristMacroActive) {
        scoreOpenPower = ratchetWristPower;
    }
    if (pinGrabState != PinGrabState::MovingWristDown) {
        devices.Motors.MOTOR_SCORE_OPEN.set(scoreOpenPower);
    }

    int scorerollPower = intakePower;
    if (pinGrabState == PinGrabState::RollingIntake) {
        if (pros::millis() - scoreRollMacroStart >= scoreRollMacroTimeMs) {
            scorerollPower = 0;
        } else {
            scorerollPower = scoreRollMacroPower;
        }
    } else if (seekingColor) {
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
    previousX = x;
    previousY = y;

    devices.Motors.MOTOR_SCORE_ROLL.set(scorerollPower);
}
