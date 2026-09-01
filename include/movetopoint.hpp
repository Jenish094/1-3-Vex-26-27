#pragma once
#include "helpers.hpp"

// Pose: represents a target position based on tracking wheel measurements
struct Pose {
    float x;       // target VERT (vertical wheel) position in inches
    float y;       // target HORZ (horizontal wheel) position in inches
    float heading; // heading in radians (for reference only)
    
    Pose(void) : x(0), y(0), heading(0) {}
    Pose(float x, float y) : x(x), y(y), heading(0) {}
    Pose(float x, float y, float heading) : x(x), y(y), heading(heading) {}
};

// Movement to a target pose using tracking wheel feedback with closed-loop control
// maxPower: maximum motor power (-127 to 127)
// timeout_ms: maximum time to spend moving (0 = no timeout, default 5000ms = 5 seconds)
// tolerance_in: position tolerance in inches for BOTH vert and horz (stop when both within this tolerance)
void moveToPoint(const Pose& target, int maxPower = 80, int timeout_ms = 5000, float tolerance_in = 0.05f);

// Move to point using tracking wheel positions
// vert: target vertical wheel position in inches
// horz: target horizontal wheel position in inches
void moveToPoint(float vert, float horz, int maxPower = 80, int timeout_ms = 5000, float tolerance_in = 0.05f);

// Print current VERT and HORZ tracking wheel positions to brain screen (use for recording coordinates)
// Simply call and read the screen, then pass those values to moveToPoint()
void printCurrentPosition(void);
