#ifndef ODOM_HPP
#define ODOM_HPP

#include "api.h"
#include "helpers.hpp"
#include "main.h"

// =====================================================================
// Odometry for a single vertical (forward) tracking wheel, a single
// horizontal (sideways) tracking wheel, and the IMU for heading.
//
// This replaced the old 3-wheel (2x vertical + 1x horizontal) setup,
// which computed heading purely from the difference between the two
// parallel wheels. With only one vertical wheel that trick no longer
// works, so heading now comes from the IMU instead, and both tracking
// wheels are corrected for the arc they sweep out as the robot turns.
// =====================================================================

// --- Tracking wheel hardware ---
// Diameter of the (omni) tracking wheels in inches. Update this to match
// the wheels actually mounted (2.75" and 2.00" are the common VEX sizes).
const float odomWheelDiameterIn = 2.0f;
const float odomWheelCircumferenceIn = odomWheelDiameterIn * PI;

// pros::Rotation::get_position() returns centidegrees (hundredths of a
// degree), so this converts a raw reading straight to radians.
const float centideg2rad = (PI / 180.0f) / 100.0f;
const float deg2rad = PI / 180.0f;

// --- Tracking center offsets (inches) ---
// Perpendicular distance from the robot's turning center to each
// tracking wheel's contact patch. These are NOT the same numbers as the
// old 6-motor/3-wheel bot and MUST be re-measured on the new frame:
//   - verticalOdomOffsetInches:   left(+) / right(-) distance from the
//                                 turning center to the vertical wheel.
//   - horizontalOdomOffsetInches: forward(+) / back(-) distance from the
//                                 turning center to the horizontal wheel.
// If odom position drifts while spinning in place on the spot, these
// two constants are the first thing to recheck.
const float verticalOdomOffsetInches = 1.9375f;   // TODO: verify on new frame
const float horizontalOdomOffsetInches = 0.0f;    // TODO: measure on new frame

void odomCalcPos(void);
void odomReset(void);
void brainScreenManager(void);

extern std::atomic<float> globalHeading;          // current heading, radians, CCW+
extern std::atomic<float> globalHeadingAtLastReset;

extern float lastVertical;
extern float lastHorizontal;

extern std::atomic<float> globalXIn;
extern std::atomic<float> globalYIn;
extern std::atomic<float> lastGlobalXIn;
extern std::atomic<float> lastGlobalYIn;

extern std::atomic<float> verticalPosIn;
extern std::atomic<float> horizontalPosIn;

extern std::atomic<float> cascadeEncoderPos;

extern std::atomic<int> dist;

#endif
