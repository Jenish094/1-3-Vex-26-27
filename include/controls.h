#pragma once
#include "main.h"

enum class ControlScheme {
    R2L2,
    DSA
};

extern ControlScheme currentScheme;
void controlsInit();
bool controlsUpdate();
void runDriverControls();