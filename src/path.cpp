#include "path.hpp"
#include "hardware.hpp"
#include "main.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

namespace {

struct PathPoint {
        double x;
        double y;
        double speed;
};

double clampValue(double value, double minimum, double maximum) {
        return std::max(minimum, std::min(value, maximum));
}

bool parsePath(const char *pathText, std::vector<PathPoint> &points) {
        if (pathText == nullptr)
                return false;

        std::istringstream input(pathText);
        std::string line;
        while (std::getline(input, line)) {
                const std::size_t first = line.find_first_not_of(" \t\r\n");
                if (first == std::string::npos || line[first] == '#' ||
                    (line[first] == '/' && first + 1 < line.size() && line[first + 1] == '/'))
                        continue;

                for (char &character : line) {
                        if (character == ',')
                                character = ' ';
                }

                std::istringstream values(line);
                PathPoint point{};
                point.speed = 100.0;
                if (!(values >> point.x >> point.y) || !(std::isfinite(point.x) &&
                    std::isfinite(point.y)))
                        continue;
                values >> point.speed;
                if (!std::isfinite(point.speed))
                        point.speed = 100.0;
                point.speed = clampValue(point.speed, 0.0, 100.0);
                points.push_back(point);
        }
        return points.size() >= 2;
}

double distanceBetween(const PathPoint &first, const PathPoint &second) {
        return std::hypot(second.x - first.x, second.y - first.y);
}

} // namespace

bool Robot::follow(const char *pathText, double timeoutMs) {
        const auto stopDrivetrain = [] {
                devices.MGroups.DT_L.set(0);
                devices.MGroups.DT_R.set(0);
        };

        std::vector<PathPoint> points;
        if (!std::isfinite(timeoutMs) || timeoutMs <= 0.0 || !parsePath(pathText, points)) {
                stopDrivetrain();
                std::printf("Robot::follow: expected at least two x,y,speed points\n");
                return false;
        }

        const double firstDeltaX = points[1].x - points[0].x;
        const double firstDeltaY = points[1].y - points[0].y;
        if (std::hypot(firstDeltaX, firstDeltaY) < 1e-6) {
                stopDrivetrain();
                std::printf("Robot::follow: first two path points must differ\n");
                return false;
        }

        const double initialHeading = std::atan2(-firstDeltaX, firstDeltaY);
        odomSetPose(points.front().x, points.front().y, initialHeading);
        startOdometryTask();
        isAuton = true;

        std::vector<double> cumulativeLength(points.size(), 0.0);
        for (std::size_t i = 1; i < points.size(); ++i)
                cumulativeLength[i] =
                    cumulativeLength[i - 1] + distanceBetween(points[i - 1], points[i]);
        const double totalLength = cumulativeLength.back();
        const std::uint32_t startTime = pros::millis();
        constexpr double lookaheadInches = 8.0;

        while (pros::millis() - startTime < timeoutMs) {
                const double x = globalXIn.load();
                const double y = globalYIn.load();
                const double heading = globalHeading.load();

                double closestDistance = std::numeric_limits<double>::max();
                double pathPosition = 0.0;
                std::size_t closestSegment = 0;
                for (std::size_t i = 1; i < points.size(); ++i) {
                        const double segmentX = points[i].x - points[i - 1].x;
                        const double segmentY = points[i].y - points[i - 1].y;
                        const double segmentLengthSquared = segmentX * segmentX + segmentY * segmentY;
                        if (segmentLengthSquared < 1e-9)
                                continue;
                        const double ratio = clampValue(
                            ((x - points[i - 1].x) * segmentX + (y - points[i - 1].y) * segmentY) /
                                segmentLengthSquared,
                            0.0, 1.0);
                        const double nearestX = points[i - 1].x + ratio * segmentX;
                        const double nearestY = points[i - 1].y + ratio * segmentY;
                        const double distance = std::hypot(x - nearestX, y - nearestY);
                        if (distance < closestDistance) {
                                closestDistance = distance;
                                closestSegment = i - 1;
                                pathPosition = cumulativeLength[i - 1] +
                                    ratio * std::sqrt(segmentLengthSquared);
                        }
                }

                const double targetPosition = std::min(totalLength, pathPosition + lookaheadInches);
                std::size_t targetSegment = closestSegment;
                while (targetSegment + 1 < points.size() &&
                       cumulativeLength[targetSegment + 1] < targetPosition)
                        ++targetSegment;
                targetSegment = std::min(targetSegment, points.size() - 2);
                const double segmentLength = distanceBetween(points[targetSegment],
                                                              points[targetSegment + 1]);
                const double targetRatio = segmentLength < 1e-9
                    ? 0.0
                    : clampValue((targetPosition - cumulativeLength[targetSegment]) /
                                     segmentLength,
                                 0.0, 1.0);
                const double targetX = points[targetSegment].x +
                    targetRatio * (points[targetSegment + 1].x - points[targetSegment].x);
                const double targetY = points[targetSegment].y +
                    targetRatio * (points[targetSegment + 1].y - points[targetSegment].y);
                const double targetDistance = std::hypot(targetX - x, targetY - y);

                if (std::hypot(points.back().x - x, points.back().y - y) <= 1.5 &&
                    pathPosition >= totalLength - 2.0)
                        break;

                const double deltaX = targetX - x;
                const double deltaY = targetY - y;
                const double rightError = deltaX * std::cos(heading) + deltaY * std::sin(heading);
                const double speedLimit = clampValue(points[targetSegment].speed * 1.27, 15.0, 127.0);
                const double basePower = clampValue(targetDistance * 4.0, 15.0, speedLimit);
                const double turnPower = clampValue(rightError * 4.0,
                                                    -speedLimit * 0.7, speedLimit * 0.7);
                devices.MGroups.DT_L.set(static_cast<int>(std::lround(basePower + turnPower)));
                devices.MGroups.DT_R.set(static_cast<int>(std::lround(basePower - turnPower)));
                pros::delay(10);
        }

        stopDrivetrain();
        isAuton = false;
        return true;
}