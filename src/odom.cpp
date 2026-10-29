#include "odom.hpp"
#include "hardware.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>

namespace {
constexpr double degreesToRadians = ODOM_PI / 180.0;
constexpr double centidegreesPerRevolution = 36000.0;
constexpr double odomPositionToleranceIn = 0.25;
constexpr std::uint32_t odomSettleTimeMs = 150;
constexpr double driveKp = 5.0;
constexpr double driveKi = 0.45;
constexpr double driveKd = 0.5;
constexpr double headingKp = 1.0;
std::atomic<bool> odometryResetRequested(false);

double clampValue(double value, double minimum, double maximum) {
        return std::max(minimum, std::min(value, maximum));
}

double wheelPositionInches(std::int32_t centidegrees) {
        return static_cast<double>(centidegrees) * odomWheelCircumferenceIn /
               centidegreesPerRevolution;
}

double wrappedHeadingDeltaDegrees(double current, double previous) {
        double delta = current - previous;
        if (delta > 180.0)
                delta -= 360.0;
        else if (delta < -180.0)
                delta += 360.0;
        return delta;
}

double sinc(double angle) {
        if (std::abs(angle) < 1e-6)
                return 1.0 - (angle * angle) / 24.0;
        return 2.0 * std::sin(angle * 0.5) / angle;
}

} // namespace

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

std::atomic<float> cascadeEncoderPos(0.0f);

std::atomic<int> dist(0);
Robot robot;

void odomCalcPos(void) {
        double previousVerticalIn = wheelPositionInches(devices.Odom.VerticalWheel->get_position());
        double previousHorizontalIn = wheelPositionInches(devices.Odom.HorizontalWheel->get_position());
        double previousImuHeadingDeg = devices.Sensors.IMU->get_heading();
        double previousHeading = globalHeading.load();

        verticalPosIn = static_cast<float>(previousVerticalIn);
        horizontalPosIn = static_cast<float>(previousHorizontalIn);
        lastVertical = static_cast<float>(previousVerticalIn);
        lastHorizontal = static_cast<float>(previousHorizontalIn);

        while (true) {
                const double currentVerticalIn =
                    wheelPositionInches(devices.Odom.VerticalWheel->get_position());
                const double currentHorizontalIn =
                    wheelPositionInches(devices.Odom.HorizontalWheel->get_position());
                const double currentImuHeadingDeg = devices.Sensors.IMU->get_heading();

                if (odometryResetRequested.exchange(false)) {
                        previousVerticalIn = currentVerticalIn;
                        previousHorizontalIn = currentHorizontalIn;
                        previousImuHeadingDeg = currentImuHeadingDeg;
                        previousHeading = globalHeadingAtLastReset.load();
                        verticalPosIn = static_cast<float>(currentVerticalIn);
                        horizontalPosIn = static_cast<float>(currentHorizontalIn);
                        lastVertical = static_cast<float>(currentVerticalIn);
                        lastHorizontal = static_cast<float>(currentHorizontalIn);
                        globalHeading = static_cast<float>(previousHeading);
                        pros::delay(10);
                        continue;
                }

                verticalPosIn = static_cast<float>(currentVerticalIn);
                horizontalPosIn = static_cast<float>(currentHorizontalIn);

                const double deltaVerticalIn = currentVerticalIn - previousVerticalIn;
                const double deltaHorizontalIn = currentHorizontalIn - previousHorizontalIn;
                const double deltaTheta =
                    -wrappedHeadingDeltaDegrees(currentImuHeadingDeg, previousImuHeadingDeg) *
                    degreesToRadians;
                const double currentHeading = previousHeading + deltaTheta;

                // Remove the distance each offset wheel sweeps while the robot turns.
                const double arcScale = sinc(deltaTheta);
                const double localForwardIn =
                    arcScale * (deltaVerticalIn + verticalOdomOffsetInches * deltaTheta);
                const double localSideIn =
                    arcScale * (-deltaHorizontalIn + horizontalOdomOffsetInches * deltaTheta);

                const double averageHeading = previousHeading + deltaTheta * 0.5;
                // Heading is CCW-positive, zero faces +Y, and local side is +right:
                // [dX; dY] = [-sin(h) cos(h); cos(h) sin(h)] * [forward; side].
                const double globalDeltaX =
                    -localForwardIn * std::sin(averageHeading) +
                    localSideIn * std::cos(averageHeading);
                const double globalDeltaY =
                    localForwardIn * std::cos(averageHeading) +
                    localSideIn * std::sin(averageHeading);

                lastGlobalXIn = globalXIn.load();
                lastGlobalYIn = globalYIn.load();
                globalXIn = static_cast<float>(globalXIn.load() + globalDeltaX);
                globalYIn = static_cast<float>(globalYIn.load() + globalDeltaY);
                globalHeading = static_cast<float>(currentHeading);

                previousVerticalIn = currentVerticalIn;
                previousHorizontalIn = currentHorizontalIn;
                previousImuHeadingDeg = currentImuHeadingDeg;
                previousHeading = currentHeading;
                lastVertical = static_cast<float>(currentVerticalIn);
                lastHorizontal = static_cast<float>(currentHorizontalIn);

                pros::delay(10);
        }
}

void startOdometryTask(void) {
        static pros::Task odometryTask([] { odomCalcPos(); }, "Odometry");
        (void)odometryTask;
}

void odomReset(void) {
        devices.Odom.VerticalWheel->reset_position();
        devices.Odom.HorizontalWheel->reset_position();
        devices.Sensors.IMU->tare_rotation();
        devices.Sensors.CascadeEncoder->reset_position();

        lastVertical = 0.0f;
        lastHorizontal = 0.0f;
        verticalPosIn = 0.0f;
        horizontalPosIn = 0.0f;
        cascadeEncoderPos = 0.0f;

        const float heading = globalHeading.load();
        globalHeadingAtLastReset = heading;
        globalHeading = heading;
        globalXIn = 0.0f;
        globalYIn = 0.0f;
        lastGlobalXIn = 0.0f;
        lastGlobalYIn = 0.0f;
        odometryResetRequested = true;
}

void odomSetPose(double xIn, double yIn, double headingRadians) {
        devices.Odom.VerticalWheel->reset_position();
        devices.Odom.HorizontalWheel->reset_position();
        devices.Sensors.IMU->tare_rotation();

        lastVertical = 0.0f;
        lastHorizontal = 0.0f;
        verticalPosIn = 0.0f;
        horizontalPosIn = 0.0f;
        globalHeadingAtLastReset = static_cast<float>(headingRadians);
        globalHeading = static_cast<float>(headingRadians);
        globalXIn = static_cast<float>(xIn);
        globalYIn = static_cast<float>(yIn);
        lastGlobalXIn = static_cast<float>(xIn);
        lastGlobalYIn = static_cast<float>(yIn);
        odometryResetRequested = true;
}

void driveInches(double targetInches, double maxSpeed, double timeoutMs) {
        const auto stopDrivetrain = [] {
                devices.MGroups.DT_L.set(0);
                devices.MGroups.DT_R.set(0);
        };

        if (!std::isfinite(targetInches) || !std::isfinite(maxSpeed) ||
            !std::isfinite(timeoutMs) || maxSpeed <= 0.0 || timeoutMs <= 0.0) {
                stopDrivetrain();
                return;
        }

        startOdometryTask();

        const double speedLimit = clampValue(maxSpeed, 0.0, 127.0);
        const std::uint32_t timeout = static_cast<std::uint32_t>(
            std::min(timeoutMs, static_cast<double>(std::numeric_limits<std::uint32_t>::max())));
        const std::uint32_t startTime = pros::millis();
        const double startX = globalXIn.load();
        const double startY = globalYIn.load();
        const double startHeading = globalHeading.load();
        const double startImuRotationDeg = devices.Sensors.IMU->get_rotation();
        const double headingCorrectionLimit = speedLimit * 0.4;
        const double driveDirection = targetInches < 0.0 ? -1.0 : 1.0;

        double integral = 0.0;
        double previousError = targetInches;
        std::uint32_t previousTime = startTime;
        std::uint32_t settledSince = 0;
        bool settling = false;

        while (pros::millis() - startTime < timeout) {
                const double deltaX = globalXIn.load() - startX;
                const double deltaY = globalYIn.load() - startY;
                const double travelledInches =
                    -deltaX * std::sin(startHeading) + deltaY * std::cos(startHeading);
                const double error = targetInches - travelledInches;
                const std::uint32_t now = pros::millis();
                const double dt = std::max(0.01, (now - previousTime) / 1000.0);

                double basePower = 0.0;
                if (std::abs(error) <= odomPositionToleranceIn) {
                        if (!settling) {
                                settledSince = now;
                                settling = true;
                        } else if (now - settledSince >= odomSettleTimeMs) {
                                break;
                        }
                        integral = 0.0;
                } else {
                        settling = false;
                        integral = clampValue(integral + error * dt, -20.0, 20.0);
                        const double derivative = (error - previousError) / dt;
                        basePower = clampValue(
                            driveKp * error + driveKi * integral + driveKd * derivative,
                            -speedLimit, speedLimit);
                }

                const double currentImuRotationDeg = devices.Sensors.IMU->get_rotation();
                const double headingErrorDeg = startImuRotationDeg - currentImuRotationDeg;
                const double headingCorrection = clampValue(
                    headingKp * headingErrorDeg,
                    -headingCorrectionLimit, headingCorrectionLimit);
                const double correctionDirection =
                    basePower == 0.0 ? driveDirection : (basePower < 0.0 ? -1.0 : 1.0);
                const double leftPower = clampValue(
                    basePower + correctionDirection * headingCorrection, -speedLimit, speedLimit);
                const double rightPower = clampValue(
                    basePower - correctionDirection * headingCorrection, -speedLimit, speedLimit);

                devices.MGroups.DT_L.set(static_cast<int>(std::lround(leftPower)));
                devices.MGroups.DT_R.set(static_cast<int>(std::lround(rightPower)));

                previousError = error;
                previousTime = now;
                pros::delay(10);
        }

        stopDrivetrain();
}

void Robot::move(double inches, double maxSpeed, double timeoutMs) {
        driveInches(inches * 3.0, maxSpeed, timeoutMs);
}

void Robot::turn(double degrees, double maxSpeed, double timeoutMs) {
        const auto stopDrivetrain = [] {
                devices.MGroups.DT_L.set(0);
                devices.MGroups.DT_R.set(0);
        };

        if (!std::isfinite(degrees) || !std::isfinite(maxSpeed) ||
            !std::isfinite(timeoutMs) || maxSpeed <= 0.0 || timeoutMs <= 0.0) {
                stopDrivetrain();
                return;
        }

        startOdometryTask();

        const double speedLimit = clampValue(maxSpeed, 0.0, 127.0);
        const std::uint32_t timeout = static_cast<std::uint32_t>(
            std::min(timeoutMs, static_cast<double>(std::numeric_limits<std::uint32_t>::max())));
        const std::uint32_t startTime = pros::millis();
        const double targetHeading =
            static_cast<double>(globalHeading.load()) - degrees * degreesToRadians;
        const double headingTolerance = 2.0 * degreesToRadians;
        const double headingKpDegrees = 1.2;
        const std::uint32_t settleTimeMs = 150;
        std::uint32_t settledSince = 0;
        bool settling = false;

        while (pros::millis() - startTime < timeout) {
                const double headingError = std::remainder(
                    targetHeading - static_cast<double>(globalHeading.load()),
                    2.0 * ODOM_PI);

                if (std::abs(headingError) <= headingTolerance) {
                        if (!settling) {
                                settledSince = pros::millis();
                                settling = true;
                        } else if (pros::millis() - settledSince >= settleTimeMs) {
                                break;
                        }
                        stopDrivetrain();
                } else {
                        settling = false;
                        double turnPower = clampValue(
                            headingError / degreesToRadians * headingKpDegrees,
                            -speedLimit, speedLimit);
                        if (std::abs(turnPower) < 12.0)
                                turnPower = std::copysign(12.0, turnPower);

                        // Positive degrees means right; negative degrees means left.
                        devices.MGroups.DT_L.set(static_cast<int>(std::lround(-turnPower)));
                        devices.MGroups.DT_R.set(static_cast<int>(std::lround(turnPower)));
                }

                pros::delay(10);
        }

        stopDrivetrain();
}

void brainScreenManager(void) {
        while (true) {
                pros::screen::erase();

                pros::screen::print(pros::E_TEXT_MEDIUM, 0, "VERT (in): %.4f", verticalPosIn.load());
                pros::screen::print(pros::E_TEXT_MEDIUM, 1, "HORZ (in): %.4f", horizontalPosIn.load());
                pros::screen::print(pros::E_TEXT_MEDIUM, 3, "X (In): %.4f", globalXIn.load());
                pros::screen::print(pros::E_TEXT_MEDIUM, 4, "Y (In): %.4f", globalYIn.load());
                pros::screen::print(pros::E_TEXT_MEDIUM, 5, "HDG (deg): %.4f",
                                    globalHeading.load() / degreesToRadians);

                const float cascadeRotation = devices.Sensors.CascadeEncoder->get_position();
                cascadeEncoderPos = cascadeRotation;
                pros::screen::print(pros::E_TEXT_MEDIUM, 6, "CASCADE (deg): %.4f", cascadeRotation);

                pros::delay(77);
        }
}
