#include "macro.hpp"

void FlipToggleHeight::Run(int timeout_millis)
{
        (void)timeout_millis;

        const float x = globalXIn.load();
        const float y = globalYIn.load();
        constexpr float centerY = 65.0f;
        constexpr float radius = 65.0f;

        if (std::abs(x) + std::abs(y - centerY) > radius) {
                return;
        }

        auto& gate = devices.Motors.MOTOR_SCORE_OPEN;
        gate.set(-127);
        pros::delay(900);
        gate.set(0);
        gate.set(127);
        pros::delay(450);
        gate.set(0);
}

void CascadeHeight() {}
