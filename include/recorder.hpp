#pragma once

#include "liblvgl/lvgl.h"

namespace recorder {

bool is_recording();
bool start();
void stop();
void toggle();
void create_button(lv_obj_t *parent);

} // namespace recorder
