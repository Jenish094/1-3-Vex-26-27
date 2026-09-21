#ifndef DRIVER_HPP
#define DRIVER_HPP

#include "api.h"
#include "hardware.hpp"
#include "odom.hpp"
#include "vex_macro.hpp"
#include <cmath>

extern std::atomic<bool> buttonAState;
extern std::atomic<bool> buttonBState;
extern std::atomic<bool> buttonXState;
extern std::atomic<bool> buttonYState;
extern std::atomic<bool> buttonUpState;
extern std::atomic<bool> buttonDownState;
extern std::atomic<bool> buttonLeftState;
extern std::atomic<bool> buttonRightState;

extern std::atomic<bool> bumperLeftFrontState;
extern std::atomic<bool> bumperLeftBackState;
extern std::atomic<bool> bumperRightFrontState;
extern std::atomic<bool> bumperRightBackState;

extern std::atomic<int> axisLeftX;
extern std::atomic<int> axisLeftY;
extern std::atomic<int> axisRightX;
extern std::atomic<int> axisRightY;

void driverReadInput(void);
void driverApplyInput(void);
void driverScreenManager(void);
void driverIntakeManager(void);

bool isNoBumpersPressed(void);
void spinLowSS(int pow);
void spinHighSS(int pow);

bool isAnyMotorTempAbove(float temp);
bool isAnyDriveMotorTempAbove(float temp);

int calcSpeedKmPerH(void);
void driveForTime(int power, int millis);

#endif
