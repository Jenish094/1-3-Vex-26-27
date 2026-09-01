#include "odom.hpp"

std::atomic<float> globalHeading(0.0f);
std::atomic<float> globalHeadingAtLastReset(0.0f);

float lastVertical = 0.0f;
float lastHorizontal = 0.0f;

std::atomic<float> globalXIn(0.0f);
std::atomic<float> globalYIn(0.0f);
std::atomic<float> lastGlobalXIn(0.0f);
std::atomic<float> lastGlobalYIn(0.0f);

std::atomic<float> verticalPosIn(0.0f);
std::atomic<float> horizontalPosIn(0.0f);

std::atomic<int> dist(0); // placeholder / diagnostic, see odomUpdate() in main.cpp

// Single combined odom task: reads both tracking wheels + the IMU, then
// integrates position with an arc-correction so the readings stay
// accurate while turning (not just driving straight).
void odomCalcPos(void) {
        float prevGlobalHeading = globalHeadingAtLastReset.load();

        do {
                // --- Read sensors ---
                float verticalTicks = devices.Odom.VerticalWheel->get_position();
                float horizontalTicks = devices.Odom.HorizontalWheel->get_position();

                verticalPosIn = verticalTicks * centideg2rad * (odomWheelDiameterIn * 0.5f);
                horizontalPosIn = horizontalTicks * centideg2rad * (odomWheelDiameterIn * 0.5f);

                // pros::Imu::get_rotation() is continuous (doesn't wrap at 360 like
                // get_heading()) and increases clockwise; negate it so increasing
                // globalHeading means turning counter-clockwise, matching sin/cos.
                float imuRotationDeg = devices.Sensors.IMU->get_rotation();
                float currentHeading = globalHeadingAtLastReset.load() - (imuRotationDeg * deg2rad);
                globalHeading = currentHeading;

                // --- Compute deltas since last loop ---
                float currentVertical = verticalPosIn.load();
                float currentHorizontal = horizontalPosIn.load();

                float deltaVerticalIn = currentVertical - lastVertical;
                float deltaHorizontalIn = currentHorizontal - lastHorizontal;

                lastVertical = currentVertical;
                lastHorizontal = currentHorizontal;

                float deltaTheta = currentHeading - prevGlobalHeading;

                // --- Arc-correct the local displacement ---
                // Each wheel doesn't just measure translation; while the robot
                // turns, a wheel offset from the turning center also sweeps out
                // extra arc length. This backs that out so straight-line
                // distance is recovered even mid-turn.
                float localForwardIn;
                float localSideIn;

                if (deltaTheta == 0.0f) {
                        localForwardIn = deltaVerticalIn;
                        localSideIn = deltaHorizontalIn;
                } else {
                        float chord = 2.0f * sin(deltaTheta * 0.5f);
                        localForwardIn = chord * ((deltaVerticalIn / deltaTheta) + verticalOdomOffsetInches);
                        localSideIn = chord * ((deltaHorizontalIn / deltaTheta) + horizontalOdomOffsetInches);
                }

                // --- Rotate the local displacement into the global frame ---
                // Convention: heading 0 faces +Y, and increasing (CCW) heading
                // rotates "forward" toward -X. If X/Y end up swapped or mirrored
                // on the field during testing, flip the signs below to match.
                float avgHeading = prevGlobalHeading + (deltaTheta * 0.5f);

                float globalDeltaX = (localForwardIn * sin(avgHeading)) + (localSideIn * cos(avgHeading));
                float globalDeltaY = (localForwardIn * cos(avgHeading)) - (localSideIn * sin(avgHeading));

                lastGlobalXIn = globalXIn.load();
                lastGlobalYIn = globalYIn.load();
                prevGlobalHeading = currentHeading;

                globalXIn = globalXIn.load() + globalDeltaX;
                globalYIn = globalYIn.load() + globalDeltaY;

                pros::delay(5); // ~200Hz
        } while (true);
}

void odomReset(void) {
        devices.Odom.VerticalWheel->reset_position();
        devices.Odom.HorizontalWheel->reset_position();
        devices.Sensors.IMU->tare_rotation();

        lastVertical = 0.0f;
        lastHorizontal = 0.0f;

        verticalPosIn = 0.0f;
        horizontalPosIn = 0.0f;

        globalHeading = globalHeadingAtLastReset.load();

        globalXIn = 0.0f;
        globalYIn = 0.0f;
        lastGlobalXIn = 0.0f;
        lastGlobalYIn = 0.0f;
}

void brainScreenManager(void) {
        do {
                pros::screen::erase();

                pros::screen::print(pros::E_TEXT_MEDIUM, 0, "VERT (in): %.4f", verticalPosIn.load());
                pros::screen::print(pros::E_TEXT_MEDIUM, 1, "HORZ (in): %.4f", horizontalPosIn.load());

                pros::screen::print(pros::E_TEXT_MEDIUM, 3, "X (In): %.4f", globalXIn.load());
                pros::screen::print(pros::E_TEXT_MEDIUM, 4, "Y (In): %.4f", globalYIn.load());
                pros::screen::print(pros::E_TEXT_MEDIUM, 5, "HDG (deg): %.4f", globalHeading.load() * (180.0f / PI));

                pros::delay(77);
        } while (true);
}
