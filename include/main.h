#ifndef _PROS_MAIN_H_
#define _PROS_MAIN_H_

#define PROS_USE_SIMPLE_NAMES
#define PROS_USE_LITERALS

#include "api.h"

#ifdef __cplusplus
extern "C" {
#endif
        void autonomous(void);
        void initialize(void);
        void disabled(void);
        void competition_initialize(void);
        void opcontrol(void);
        void odomUpdate(void);



#ifdef __cplusplus
}
#endif

#ifdef __cplusplus
#include <math.h>
#include <vector>
#include <string>
#endif

#include "helpers.hpp"
#include "odom.hpp"
#include "autonSelector.hpp"
#include "controls.h"
#include "recorder.hpp"

class Odom;

const float DRIVEKP = 0;
const float DRIVEKI = 0;
const float DRIVEKD = 0;

const float TURNKP = 0;
const float TURNKI = 0;
const float TURNKD = 0;

const float WHEEL_DIAMETER_IN = 2.75;
const float WHEEL_DIAMETER_MM = WHEEL_DIAMETER_IN * IN_TO_MM;
const float WHEEL_CIRCUMFERENCE = PI * WHEEL_DIAMETER_MM;

// AutonSelector Selector = AutonSelector();
extern bool isAuton;
extern bool allianceIsRed;

void initializeRecorderTask(void);

#endif  // _PROS_MAIN_H_
