#include "functions.h"
#include "controls.h"
#include "hardware.hpp"
#include <cmath>
#include <string>

namespace {
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
bool previousLeft = false;
bool previousUp = false;
bool previousRight = false;
}

void functionsInit() {
    goalMode = GoalMode::Neutral;
    scoreMode = ScoreMode::Pin;
    cascadeLevel = 0;
    previousA = false;
    previousB = false;
    previousLeft = false;
    previousUp = false;
    previousRight = false;
    updateSecondaryControllerScreen();
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
    controller.clear();
    controller.set_text(0, 0, std::string("goal: ") + goalText);
    controller.set_text(1, 0, std::string("scr: ") + scoreText);
}

void updateCascadeAutomation() {
    if (!devices.SECONDARY_CONTROLLER || !devices.Sensors.CascadeEncoder) {
        return;
    }

    pros::Controller& secondary = *devices.SECONDARY_CONTROLLER;
    bool left = secondary.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT);
    bool up = secondary.get_digital(pros::E_CONTROLLER_DIGITAL_UP);
    bool right = secondary.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT);
    bool b = secondary.get_digital(pros::E_CONTROLLER_DIGITAL_B);
    bool a = secondary.get_digital(pros::E_CONTROLLER_DIGITAL_A);

    if (left && !previousLeft) {
        goalMode = GoalMode::Neutral;
        updateSecondaryControllerScreen();
    }
    if (up && !previousUp) {
        goalMode = GoalMode::Centre;
        updateSecondaryControllerScreen();
    }
    if (right && !previousRight) {
        goalMode = GoalMode::Alliance;
        updateSecondaryControllerScreen();
    }
    if (b && !previousB) {
        scoreMode = scoreMode == ScoreMode::Pin ? ScoreMode::Cup : ScoreMode::Pin;
        updateSecondaryControllerScreen();
    }
    if (a && !previousA && cascadeLevel < 10) {
        ++cascadeLevel;
    }

    previousLeft = left;
    previousUp = up;
    previousRight = right;
    previousB = b;
    previousA = a;

    int manualPower = (devices.PRIMARY_CONTROLLER->get_digital(pros::E_CONTROLLER_DIGITAL_L2) ? 127 : 0)
        - (devices.PRIMARY_CONTROLLER->get_digital(pros::E_CONTROLLER_DIGITAL_L1) ? 127 : 0);
    if (manualPower != 0) {
        return;
    }

    const float* levelDistances = scoreMode == ScoreMode::Pin
        ? PIN_LEVEL_DISTANCES
        : CUP_LEVEL_DISTANCES;
    float error = levelDistances[cascadeLevel] - devices.Sensors.CascadeEncoder->get_position();
    const float tolerance = 50.0f;
    const float proportionalGain = 0.05f;

    if (std::fabs(error) <= tolerance) {
        devices.Motors.MOTOR_CASCADE_11WR.set(0);
        devices.Motors.MOTOR_CASCADE_11WL.set(0);
        return;
    }

    int power = static_cast<int>(error * proportionalGain);
    power = power > 127 ? 127 : power < -127 ? -127 : power;
    devices.Motors.MOTOR_CASCADE_11WR.set(power);
    devices.Motors.MOTOR_CASCADE_11WL.set(power);
}