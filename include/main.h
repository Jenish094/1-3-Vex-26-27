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
#ifdef __cplusplus
}
#endif

#include "odom.hpp"
#include "autonSelector.hpp"
#include "controls.h"
#include "recorder.hpp"

extern bool isAuton;

void initializeRecorderTask(void);

#endif  // _PROS_MAIN_H_
