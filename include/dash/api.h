/**
 * @file api.h
 * @brief API entrypoint for the dash library
 *
 * Includes all necessary functions to use the dash library.
 */

#pragma once

#define DASH
#define RD_VERSION_MAJOR 2
#define RD_VERSION_MINOR 3
#define RD_VERSION_PATCH 1

#include "liblvgl/lvgl.h"

// ========================== Check Compatibility ========================== //

#if (LV_VERSION_CHECK(8, 3, 0) == 0)
#error "LVGL version incompatible with dash (Requires >= 8.3.X)"
#endif

#ifndef LV_FONT_MONTSERRAT_12
#error "LVGL install incompatible with dash, missing font: Montserrat 12"
#endif

#ifndef LV_FONT_MONTSERRAT_14
#error "LVGL install incompatible with dash, missing font: Montserrat 14"
#endif

#ifndef LV_FONT_MONTSERRAT_16
#error "LVGL install incompatible with dash, missing font: Montserrat 16"
#endif

// ============================ Include Library ============================ //

#include "core.h"

#ifdef __cplusplus
#include "views/console.hpp"
#include "views/image.hpp"
#include "views/selector.hpp"
#endif