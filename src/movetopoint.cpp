#include "movetopoint.hpp"
#include "odom.hpp"
#include "hardware.hpp"
#include <cmath>

// Configuration flag - set to true to reverse the drivetrain during autonomous
bool AUTON_REVERSED = false;

// Clamp a value between min and max
static float clamp(float value, float min, float max) {
    if (value > max) return max;
    if (value < min) return min;
    return value;
}

// Normalize angle to [-PI, PI]
static float normalizeAngle(float angle) {
    while (angle > PI) angle -= 2.0f * PI;
    while (angle < -PI) angle += 2.0f * PI;
    return angle;
}

void moveToPoint(float vert, float horz, int maxPower, float heading) {
    // Get starting time for timeout (5 second max)
    unsigned long startTime = pros::millis();
    const int timeout_ms = 5000;
    const float tolerance_in = 0.05f;
    const float headingTolerance = 0.15f; // radians (~8.6 degrees)
    
    // Track frames at target
    int framesAtTarget = 0;
    const int framesNeeded = 3;  // Must be at target for 3 frames
    
    while (true) {
        // Check timeout
        if ((pros::millis() - startTime) > timeout_ms) {
            devices.MGroups.DT_L.set(0);
            devices.MGroups.DT_R.set(0);
            break;
        }
        
        // Get current position
        float currentVert = verticalPosIn.load();
        float currentHorz = horizontalPosIn.load();
        float currentHeading = globalHeading.load();
        
        // Calculate error
        float vertError = vert - currentVert;
        float horzError = horz - currentHorz;
        
        // Debug output every 5 loops
        static int debugCounter = 0;
        if (debugCounter++ % 5 == 0) {
            pros::screen::erase();
            pros::screen::print(pros::E_TEXT_MEDIUM, 0, "Target: V=%.2f H=%.2f", vert, horz);
            pros::screen::print(pros::E_TEXT_MEDIUM, 1, "Current: V=%.2f H=%.2f", currentVert, currentHorz);
            pros::screen::print(pros::E_TEXT_MEDIUM, 2, "Error: V=%.3f H=%.3f", vertError, horzError);
            pros::screen::print(pros::E_TEXT_MEDIUM, 3, "Hdg: %.2f rad (%.1f deg)", currentHeading, currentHeading * 180.0f / PI);
            
            if (heading != 0.0f) {
                float headingErr = normalizeAngle(heading - currentHeading);
                pros::screen::print(pros::E_TEXT_MEDIUM, 4, "HdgErr: %.2f deg", headingErr * 180.0f / PI);
            }
            
            pros::screen::print(pros::E_TEXT_MEDIUM, 5, "AtTarget: %d/%d", framesAtTarget, framesNeeded);
            
            if (AUTON_REVERSED) {
                pros::screen::print(pros::E_TEXT_MEDIUM, 6, "Mode: REVERSED");
            } else {
                pros::screen::print(pros::E_TEXT_MEDIUM, 6, "Mode: FORWARD");
            }
        }
        
        // Check if we're close enough to position
        bool positionReached = (std::abs(vertError) < tolerance_in && std::abs(horzError) < tolerance_in);
        
        // Check heading if specified
        bool headingReached = true;
        if (heading != 0.0f) {
            float headingError = normalizeAngle(heading - currentHeading);
            headingReached = (std::abs(headingError) < headingTolerance);
        }
        
        if (positionReached && headingReached) {
            framesAtTarget++;
            
            if (framesAtTarget >= framesNeeded) {
                devices.MGroups.DT_L.set(0);
                devices.MGroups.DT_R.set(0);
                break;
            }
        } else {
            framesAtTarget = 0;
        }
        
        float linearPower = clamp(vertError * 8.0f, -maxPower, maxPower);
        
        // Calculate angular (rotation) power if heading is specified
        float angularPower = 0.0f;
        if (heading != 0.0f) {
            float headingError = normalizeAngle(heading - currentHeading);
            angularPower = clamp(headingError * 15.0f, -maxPower * 0.5f, maxPower * 0.5f);
        }
        
        // Apply reversal if configured
        if (AUTON_REVERSED) {
            linearPower = -linearPower;
        }
        
        float leftPower;
        float rightPower;

        if (std::abs(angularPower) > std::abs(linearPower)) {
            leftPower = angularPower;
            rightPower = -angularPower;
        } else {
            leftPower = linearPower + angularPower;
            rightPower = linearPower - angularPower;
        }

        leftPower = clamp(leftPower, -127.0f, 127.0f);
        rightPower = clamp(rightPower, -127.0f, 127.0f);
        
        devices.MGroups.DT_L.set((int)leftPower);
        devices.MGroups.DT_R.set((int)rightPower);
        
        pros::delay(20);
    }
}

void printCurrentPosition(void) {
    float vert = verticalPosIn.load();
    float horz = horizontalPosIn.load();
    
    pros::screen::erase();
    pros::screen::print(pros::E_TEXT_LARGE, 0, "=== TRACKING POSITIONS ===");
    pros::screen::print(pros::E_TEXT_MEDIUM, 2, "VERT (in): %.4f", vert);
    pros::screen::print(pros::E_TEXT_MEDIUM, 3, "HORZ (in): %.4f", horz);
    pros::screen::print(pros::E_TEXT_MEDIUM, 5, "Copy to auto.cpp:");
    pros::screen::print(pros::E_TEXT_SMALL, 6, "moveToPoint(%.2f, %.2f);", vert, horz);
}
