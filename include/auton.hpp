#pragma once
#include "api.h"
#include "odom.hpp"
#include "hardware.hpp"

void auton_faceHeading(float heading, int timeout_millis);
void auton_driveDistance(int distance, int timeout_millis);
void auton_driveToPoint(float targetX, float targetY, int timeout_millis, bool backward = false);

const float turnOutputPowerMod = 0.8f; // MUST BE LOWER THAN 1, MOTORS CANNOT BE OVERCLOCKED or 💀
const float turnKp = 50.0f;
const float turnKi = 0.8f;
const float turnKd = 0.5f;

extern float turnPastError;
extern float turnPastHeading;
extern float turnIntegral;

const float driveKp = 5.0f;
const float driveKi = 0.0f;
const float driveKd = 0.0f;

extern float drivePastError;
extern float drivePastDError;
extern float driveIntegral;
extern Vector2 drivePastPos;
extern int drivePastDist;