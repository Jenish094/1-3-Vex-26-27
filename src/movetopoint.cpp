#include "movetopoint.hpp"
#include "odom.hpp"
#include "hardware.hpp"
#include <cmath>

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

void moveToPoint(const Pose& target, int maxPower, int timeout_ms, float tolerance_in) {
    // Get starting time for timeout
    unsigned long startTime = pros::millis();
    
    // PID/Proportional gains
    const float kP_linear = 0.8f;   // linear proportional gain (power per inch of error)
    const float kP_angular = 15.0f; // angular proportional gain (power per radian of error)
    
    // Thresholds
    const float headingTolerance = 0.15f; // radians (~8.6 degrees)
    
    // Track frames at target to avoid noise
    int framesAtTarget = 0;
    const int framesNeeded = 5;  // Must be at target for 5 frames (100ms)
    
    while (true) {
        // Check timeout
        if (timeout_ms > 0 && (pros::millis() - startTime) > timeout_ms) {
            devices.MGroups.DT_L.set(0);
            devices.MGroups.DT_R.set(0);
            break;
        }
        
        // Get current state - use raw tracking wheel positions
        float currentVert = verticalPosIn.load();
        float currentHorz = horizontalPosIn.load();
        float currentHeading = globalHeading.load();
        
        // Calculate error using tracking wheel positions
        float vertError = target.x - currentVert;  // target.x = vertical target
        float horzError = target.y - currentHorz;  // target.y = horizontal target
        float distanceError = std::sqrt(vertError * vertError + horzError * horzError);
        
        // Debug output every 10 loops
        static int debugCounter = 0;
        if (debugCounter++ % 10 == 0) {
            pros::screen::erase();
            pros::screen::print(pros::E_TEXT_MEDIUM, 0, "Tgt VERT: %.2f, HORZ: %.2f", target.x, target.y);
            pros::screen::print(pros::E_TEXT_MEDIUM, 1, "Cur VERT: %.2f, HORZ: %.2f", currentVert, currentHorz);
            pros::screen::print(pros::E_TEXT_MEDIUM, 2, "Vert Err: %.4f  Horz Err: %.4f", vertError, horzError);
            pros::screen::print(pros::E_TEXT_MEDIUM, 3, "Distance: %.4f in", distanceError);
            pros::screen::print(pros::E_TEXT_MEDIUM, 4, "At target: %d/%d", framesAtTarget, framesNeeded);
        }
        
        // Check if we're close enough (both vertical and horizontal within tolerance)
        if (std::abs(vertError) < tolerance_in && std::abs(horzError) < tolerance_in) {
            framesAtTarget++;
            
            // If we've been at target long enough, exit
            if (framesAtTarget >= framesNeeded) {
                devices.MGroups.DT_L.set(0);
                devices.MGroups.DT_R.set(0);
                break;
            }
        } else {
            framesAtTarget = 0;  // Reset if we drift away
        }
        
        // Calculate desired direction to target based on tracking wheel deltas
        float desiredHeading = std::atan2(horzError, vertError);
        float headingError = normalizeAngle(desiredHeading - currentHeading);
        
        // Calculate linear and angular components
        float linearPower = clamp(distanceError * kP_linear, -maxPower, maxPower);
        float angularPower = clamp(headingError * kP_angular, -maxPower * 0.4f, maxPower * 0.4f);
        
        // Combine: forward/back + rotation
        float leftPower = clamp(linearPower + angularPower, -127.0f, 127.0f);
        float rightPower = clamp(linearPower - angularPower, -127.0f, 127.0f);
        
        devices.MGroups.DT_L.set((int)leftPower);
        devices.MGroups.DT_R.set((int)rightPower);
        
        pros::delay(20);
    }
}

void moveToPoint(float x, float y, int maxPower, int timeout_ms, float tolerance_in) {
    // Move using tracking wheel positions (vert, horz)
    Pose target(x, y, 0);  // x = vertical target, y = horizontal target
    moveToPoint(target, maxPower, timeout_ms, tolerance_in);
}

void printCurrentPosition(void) {
    float vert = verticalPosIn.load();
    float horz = horizontalPosIn.load();
    
    pros::screen::erase();
    pros::screen::print(pros::E_TEXT_LARGE, 0, "=== TRACKING POSITIONS ===");
    pros::screen::print(pros::E_TEXT_MEDIUM, 2, "VERT (in): %.4f", vert);
    pros::screen::print(pros::E_TEXT_MEDIUM, 3, "HORZ (in): %.4f", horz);
    pros::screen::print(pros::E_TEXT_MEDIUM, 5, "Copy to auto.cpp:");
    pros::screen::print(pros::E_TEXT_SMALL, 6, "moveToPoint(%.2f, %.2f)", vert, horz);
}
