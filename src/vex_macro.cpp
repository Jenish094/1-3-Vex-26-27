#include "vex_macro.hpp"

void FlipToggleHeight::Run(int timeout_millis) {
        const float xPos = globalXIn.load();
        const float yPos = globalYIn.load();

        // (0,0), (-65,65), (0,130), (65,65)
        const float zoneCenterY = 65.0f;
        const float zoneRadius = 65.0f;

        bool insideToggleZone = (std::abs(xPos) + std::abs(yPos - zoneCenterY)) <= zoneRadius;
        if (!insideToggleZone) {
                return;
        }

        devices.Motors.MOTOR_SCORE_OPEN.set(-127);
        pros::delay(900);
        devices.Motors.MOTOR_SCORE_OPEN.set(0);

        devices.Motors.MOTOR_SCORE_OPEN.set(127);
        pros::delay(450);
        devices.Motors.MOTOR_SCORE_OPEN.set(0);
}