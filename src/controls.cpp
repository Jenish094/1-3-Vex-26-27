#include "controls.h"
#include "odom.hpp"
#include <chrono>

static bool scoreopenReverseState = false;

// =====================================================================
// CASCADE AUTOMATION - 10 LEVEL HEIGHT SYSTEM
// =====================================================================
// Cascade encoder rotation distances for each level (in degrees)
// Level 0 = 0.0 degrees (retracted)
// TODO: Manually measure and fill in these values using the brain screen
static const float LEVEL_DISTANCES[11] = {
    0.0f,      // Level 0 - retracted (PLACEHOLDER)
    900.0f,    // Level 1 (PLACEHOLDER - replace with your measured value)
    1800.0f,   // Level 2 (PLACEHOLDER - replace with your measured value)
    2700.0f,   // Level 3 (PLACEHOLDER - replace with your measured value)
    3600.0f,   // Level 4 (PLACEHOLDER - replace with your measured value)
    4500.0f,   // Level 5 (PLACEHOLDER - replace with your measured value)
    5400.0f,   // Level 6 (PLACEHOLDER - replace with your measured value)
    6300.0f,   // Level 7 (PLACEHOLDER - replace with your measured value)
    7200.0f,   // Level 8 (PLACEHOLDER - replace with your measured value)
    8100.0f,   // Level 9 (PLACEHOLDER - replace with your measured value)
    9000.0f    // Level 10 (PLACEHOLDER - replace with your measured value)
};

static int cascadeCurrentLevel = 0;
static int cascadeTargetLevel = 0;
static int cascadeClickCount = 0;
static std::chrono::steady_clock::time_point cascadeLastClickTime;
static bool cascadeAutomationActive = false;

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

    // =====================================================================
    // CASCADE AUTOMATION: X button to activate/increment level
    // =====================================================================
    bool x = controller.get_digital(pros::E_CONTROLLER_DIGITAL_X);
    bool y = controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y);
    static bool previousX = false;
    static bool previousY = false;

    if (x && !previousX) {
        // X button pressed - increment click count
        auto now = std::chrono::steady_clock::now();
        
        if (!cascadeAutomationActive) {
            // First click - start new automation sequence
            cascadeClickCount = 1;
            cascadeAutomationActive = true;
            cascadeLastClickTime = now;
        } else {
            // Check if within 5 second timeout
            auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - cascadeLastClickTime);
            if (elapsed.count() < 5) {
                // Within timeout - increment click count (max 10)
                cascadeClickCount = (cascadeClickCount < 10) ? cascadeClickCount + 1 : 10;
                cascadeLastClickTime = now;
            } else {
                // Timeout exceeded - start new sequence
                cascadeClickCount = 1;
                cascadeLastClickTime = now;
            }
        }
        
        cascadeTargetLevel = cascadeClickCount;
    }
    
    if (y && !previousY) {
        // Y button pressed - reset to level 0
        cascadeClickCount = 0;
        cascadeTargetLevel = 0;
        cascadeAutomationActive = false;
    }
    
    previousX = x;
    previousY = y;
}

// =====================================================================
// CASCADE AUTOMATION MOVEMENT CONTROLLER
// =====================================================================
void cascadeAutomation() {
    float currentEncoderPos = devices.Sensors.CascadeEncoder->get_position();
    float targetEncoderPos = LEVEL_DISTANCES[cascadeTargetLevel];
    
    float error = targetEncoderPos - currentEncoderPos;
    const float TOLERANCE = 50.0f;  // Allow 50 degree tolerance
    const int MAX_POWER = 127;
    const float PROPORTIONAL_GAIN = 0.05f;  // Tune this for smooth movement
    
    if (std::abs(error) > TOLERANCE) {
        // Calculate proportional power (scales with error)
        int power = (int)(error * PROPORTIONAL_GAIN);
        power = (power > MAX_POWER) ? MAX_POWER : (power < -MAX_POWER) ? -MAX_POWER : power;
        
        devices.Motors.MOTOR_CASCADE_11WR.set(power);
        devices.Motors.MOTOR_CASCADE_11WL.set(power);
    } else {
        // Within tolerance - hold position
        devices.Motors.MOTOR_CASCADE_11WR.set(0);
        devices.Motors.MOTOR_CASCADE_11WL.set(0);
        cascadeCurrentLevel = cascadeTargetLevel;
    }
}
