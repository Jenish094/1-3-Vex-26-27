#ifndef ODOM_HPP
#define ODOM_HPP

#include "api.h"
#include <atomic>

//close enough to pi
constexpr double ODOM_PI = 3.14159265358979323846;
constexpr double odomWheelDiameterIn = 2.0;
constexpr double odomWheelCircumferenceIn = odomWheelDiameterIn * ODOM_PI;

// Offsets are from the rotation center. Positive vertical offset means left positive horizontal offset means forward.
constexpr double verticalOdomOffsetInches = 1.9375;
constexpr double horizontalOdomOffsetInches = 0.0;

void startOdometryTask(void);
void odomCalcPos(void);
void odomReset(void);
void odomSetPose(double xIn, double yIn, double headingRadians);
void driveInches(double targetInches, double maxSpeed = 100.0, double timeoutMs = 3000.0);
void brainscreen(void);

class Robot {
public:
        void move(double inches, double maxSpeed = 100.0, double timeoutMs = 3000.0);
        void turn(double degrees, double maxSpeed = 60.0, double timeoutMs = 3000.0);
        bool follow(const char *pathText, double timeoutMs = 15000.0);
};

extern Robot robot;

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
