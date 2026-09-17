#include "functions.h"
#include "controls.h"
#include "hardware.hpp"
#include <cmath>
#include <cstdio>
#include <string>

// Cascade height automation instructions (from the goat programmer here): 

// CONTROLS

// Secondary Controller:
// Left: Neutral goal mode
// Up: Centre goal mode
// Right: Alliance goal mode
// B: Toggle between pin and cup scoring
// A: Increase cascade level (up to 10)


// CONTROLLER SCREEN INFORMATION

// Modes will print on the secondary controller screen for being in alliance, neutral or centre mode and pin/cup mode.
// Cascade rotation height in float value will be printed on the primary controllers screen. 
// Set the cascade height manually using the primary controller in driver mode using L1/L2.
// Measure the PIN_LEVEL_DISTANCES and CUP_LEVEL_DISTANCES while in alliance mode.
// The cascade will automatically move to the correct height when the secondary controller is used to change the level.


// MEASUREMENT INSTRUCTIONS

// PIN MODE IS WHEN YOU"RE PLACING A CUP ON TOP OF A PIN
// Level 0 is when the goal is empty, hence pretty useless in pin mode
// Level 1 is when there is 1 pin in the goal and you are placing a cup ontop
// Level 2 is when there is 1 pin in the goal, there is a cup and another pin ontop, and you are placing a cup ontop of that pin
// Level 3-10 is the same but keeps going in that pattern.

// CUP MODE IS WHEN YOU"RE PLACING A PIN ON TOP OF A CUP
// Level 0 is when the goal is empty
// Level 1 is when there is a pin in the goal, and a cup ontop of that pin and you are placing a pin ontop of that cup
// Level 2 is when there is a pin in the goal, and a cup ontop of that pin and there is a pin ontop of that cup and another cup ontop of that pin, and you are placing a pin ontop of that cup
// Level 3-10 is the same but keeps going in that pattern.

// If you manually move the cascade by hand too much I will personally come to your house and get yo ahh (*cough* Alec Wallace)

namespace {
// These are the only distances that need to be measured. Measure them in
// alliance goal mode, because the alliance goal is the shortest goal.
const float PIN_LEVEL_DISTANCES[11] = {
    0.0f, // Level 0
    900.0f, // Level 1
    1800.0f, // Level 2
    2700.0f, // Level 3
    3600.0f, // Level 4
    4500.0f, // Level 5
    5400.0f, // Level 6
    6300.0f, // Level 7
    7200.0f, // Level 8
    8100.0f, // Level 9
    9000.0f // Level 10
};

const float CUP_LEVEL_DISTANCES[11] = {
    0.0f, // Level 0
    900.0f, // Level 1
    1800.0f, // Level 2
    2700.0f, // Level 3
    3600.0f, // Level 4
    4500.0f, // Level 5
    5400.0f, // Level 6
    6300.0f, // Level 7
    7200.0f, // Level 8
    8100.0f, // Level 9
    9000.0f // Level 10
};

// Goal base heights from the floor.
const float ALLIANCE_BASE_HEIGHT_MM = 82.5f;
const float NEUTRAL_BASE_HEIGHT_MM = 146.5f;
const float CENTRE_BASE_HEIGHT_MM = 222.7f;

// The rotation sensor reports centidegrees. Measure this once by raising the
// cascade a known distance, then replace 1.0f with encoder units per mm.
const float CASCADE_ENCODER_UNITS_PER_MM = 1.0f;

enum class GoalMode {
    Neutral,
    Centre,
    Alliance
};

enum class ScoreMode {
    Pin,
    Cup
};

GoalMode goalMode = GoalMode::Neutral;
ScoreMode scoreMode = ScoreMode::Pin;
int cascadeLevel = 0;
bool previousA = false;
bool previousB = false;
bool previousX = false;
bool previousPartnerL2 = false;
bool previousLeft = false;
bool previousUp = false;
bool previousRight = false;
bool cascadeStopped = false;
bool cascadeAutomationArmed = false;
bool primaryScreenInitialized = false;
bool secondaryScreenInitialized = false;
uint32_t movementStartTime = 0;

float goalHeightOffset() {
    if (goalMode == GoalMode::Neutral) {
        return (NEUTRAL_BASE_HEIGHT_MM - ALLIANCE_BASE_HEIGHT_MM)
            * CASCADE_ENCODER_UNITS_PER_MM;
    }
    if (goalMode == GoalMode::Centre) {
        return (CENTRE_BASE_HEIGHT_MM - ALLIANCE_BASE_HEIGHT_MM)
            * CASCADE_ENCODER_UNITS_PER_MM;
    }
    return 0.0f;
}
}

void functionsInit() {
    goalMode = GoalMode::Neutral;
    scoreMode = ScoreMode::Pin;
    cascadeLevel = 0;
    previousA = false;
    previousB = false;
    previousX = false;
    previousPartnerL2 = false;
    previousLeft = false;
    previousUp = false;
    previousRight = false;
    cascadeStopped = false;
    cascadeAutomationArmed = false;
    primaryScreenInitialized = false;
    secondaryScreenInitialized = false;
    movementStartTime = 0;
    updateSecondaryControllerScreen();
}

void updatePrimaryControllerScreen() {
    if (!devices.PRIMARY_CONTROLLER || !devices.Sensors.CascadeEncoder) {
        return;
    }

    static uint32_t lastScreenUpdate = 0;
    uint32_t now = pros::millis();
    if (now - lastScreenUpdate < 100) {
        return;
    }
    lastScreenUpdate = now;

    char rotationText[24];
    std::snprintf(
        rotationText,
        sizeof(rotationText),
        "rot: %d",
        devices.Sensors.CascadeEncoder->get_position()
    );

    pros::Controller& controller = *devices.PRIMARY_CONTROLLER;
    if (!primaryScreenInitialized) {
        controller.clear();
        primaryScreenInitialized = true;
    }
    controller.set_text(0, 0, rotationText);
}

void updateSecondaryControllerScreen() {
    if (!devices.SECONDARY_CONTROLLER) {
        return;
    }

    const char* goalText = "neutral";
    if (goalMode == GoalMode::Centre) {
        goalText = "centre";
    } else if (goalMode == GoalMode::Alliance) {
        goalText = "alliance";
    }

    const char* scoreText = scoreMode == ScoreMode::Pin ? "pin" : "cup";
    pros::Controller& controller = *devices.SECONDARY_CONTROLLER;
    char modeLine[16];
    std::snprintf(modeLine, sizeof(modeLine), "%s %s", goalText, scoreText);
    if (!secondaryScreenInitialized) {
        controller.clear();
        secondaryScreenInitialized = true;
    }
    controller.set_text(0, 0, modeLine);
}

void updateCascadeAutomation() {
    updatePrimaryControllerScreen();

    static uint32_t lastSecondaryScreenUpdate = 0;
    uint32_t now = pros::millis();
    if (now - lastSecondaryScreenUpdate >= 250) {
        updateSecondaryControllerScreen();
        lastSecondaryScreenUpdate = now;
    }

    if (!devices.SECONDARY_CONTROLLER || !devices.Sensors.CascadeEncoder) {
        return;
    }

    pros::Controller& secondary = *devices.SECONDARY_CONTROLLER;
    bool left = secondary.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT);
    bool up = secondary.get_digital(pros::E_CONTROLLER_DIGITAL_UP);
    bool right = secondary.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT);
    bool b = secondary.get_digital(pros::E_CONTROLLER_DIGITAL_B);
    bool a = secondary.get_digital(pros::E_CONTROLLER_DIGITAL_A);
    bool x = secondary.get_digital(pros::E_CONTROLLER_DIGITAL_X);
    bool partnerL2 = secondary.get_digital(pros::E_CONTROLLER_DIGITAL_L2);

    if (x && !previousX) {
        cascadeStopped = true;
        cascadeAutomationArmed = false;
        cascadeLevel = 0;
        movementStartTime = 0;
        devices.Motors.MOTOR_CASCADE_11WR.set(0);
        devices.Motors.MOTOR_CASCADE_11WL.set(0);
        updateSecondaryControllerScreen();
    }

    if (partnerL2) {
        cascadeStopped = true;
        cascadeAutomationArmed = false;
        cascadeLevel = 0;
        movementStartTime = 0;
        devices.Motors.MOTOR_CASCADE_11WR.set(0);
        devices.Motors.MOTOR_CASCADE_11WL.set(0);
        devices.Sensors.CascadeEncoder->reset_position();
        updateSecondaryControllerScreen();
    }

    if (left && !previousLeft) {
        goalMode = GoalMode::Neutral;
        movementStartTime = 0;
        updateSecondaryControllerScreen();
    }
    if (up && !previousUp) {
        goalMode = GoalMode::Centre;
        movementStartTime = 0;
        updateSecondaryControllerScreen();
    }
    if (right && !previousRight) {
        goalMode = GoalMode::Alliance;
        movementStartTime = 0;
        updateSecondaryControllerScreen();
    }
    if (b && !previousB) {
        scoreMode = scoreMode == ScoreMode::Pin ? ScoreMode::Cup : ScoreMode::Pin;
        movementStartTime = 0;
        updateSecondaryControllerScreen();
    }
    if (a && !previousA && cascadeLevel < 10) {
        ++cascadeLevel;
        cascadeStopped = false;
        cascadeAutomationArmed = true;
        movementStartTime = 0;
    }

    previousLeft = left;
    previousUp = up;
    previousRight = right;
    previousB = b;
    previousA = a;
    previousX = x;
    previousPartnerL2 = partnerL2;

    int manualPower = (devices.PRIMARY_CONTROLLER->get_digital(pros::E_CONTROLLER_DIGITAL_L2) ? 127 : 0)
        - (devices.PRIMARY_CONTROLLER->get_digital(pros::E_CONTROLLER_DIGITAL_L1) ? 127 : 0);
    if (manualPower != 0) {
        return;
    }

    if (cascadeStopped || !cascadeAutomationArmed) {
        devices.Motors.MOTOR_CASCADE_11WR.set(0);
        devices.Motors.MOTOR_CASCADE_11WL.set(0);
        return;
    }

    const float* levelDistances = scoreMode == ScoreMode::Pin
        ? PIN_LEVEL_DISTANCES
        : CUP_LEVEL_DISTANCES;
    float targetPosition = levelDistances[cascadeLevel];
    if (cascadeLevel > 0) {
        targetPosition += goalHeightOffset();
    }

    float error = targetPosition - devices.Sensors.CascadeEncoder->get_position();
    const float tolerance = 50.0f;
    const float proportionalGain = 0.20f;

    if (std::fabs(error) <= tolerance) {
        movementStartTime = 0;
        devices.Motors.MOTOR_CASCADE_11WR.set(0);
        devices.Motors.MOTOR_CASCADE_11WL.set(0);
        return;
    }

    if (movementStartTime == 0) {
        movementStartTime = pros::millis();
    }
    if (pros::millis() - movementStartTime > 15000) {
        cascadeStopped = true;
        cascadeLevel = 0;
        movementStartTime = 0;
        devices.Motors.MOTOR_CASCADE_11WR.set(0);
        devices.Motors.MOTOR_CASCADE_11WL.set(0);
        return;
    }

    int power = static_cast<int>(error * proportionalGain);
    if (power > 0 && power < 35) {
        power = 35;
    } else if (power < 0 && power > -35) {
        power = -35;
    }
    power = power > 127 ? 127 : power < -127 ? -127 : power;
    devices.Motors.MOTOR_CASCADE_11WR.set(power);
    devices.Motors.MOTOR_CASCADE_11WL.set(power);
}