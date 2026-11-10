#pragma once

#include "liblvgl/lvgl.h"

namespace recorder {

// Records the robot pose from the odometry task to the SD card.
bool is_recording();
bool is_requested();
bool start();
void stop();
void toggle();
void create_button(lv_obj_t *parent);

} // namespace recorder
