#pragma once
#include "helpers.hpp"

// ===== CONFIGURATION =====
// Set to true if the robot moves BACKWARD when moveToPoint is called
// Set to false if the robot moves FORWARD (normal)
extern bool AUTON_REVERSED;

// Pose: represents a target position based on tracking wheel measurements
struct Pose {
    float x;       // target VERT (vertical wheel) position in inches
    float y;       // target HORZ (horizontal wheel) position in inches
    float heading; // heading in radians (for reference only)
    
    Pose(void) : x(0), y(0), heading(0) {}
    Pose(float x, float y) : x(x), y(y), heading(0) {}
    Pose(float x, float y, float heading) : x(x), y(y), heading(heading) {}
};

// Simple straight-line movement to a target using tracking wheels
// Goes FORWARD by default (set AUTON_REVERSED = true to reverse)
// vert: target vertical wheel position in inches
// horz: target horizontal wheel position in inches
// maxPower: maximum motor power (-127 to 127)
// heading: target heading in radians (0 = no heading control, just move forward/backward)
void moveToPoint(float vert, float horz, int maxPower = 80, float heading = 0.0f);

// Print current VERT and HORZ tracking wheel positions to brain screen (use for recording coordinates)
// Simply call and read the screen, then pass those values to moveToPoint()
void printCurrentPosition(void);
