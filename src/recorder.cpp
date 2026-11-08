#include "recorder.hpp"

#include "robot_hardware.hpp"
#include "odom.hpp"
#include "pros/rtos.hpp"

#include <atomic>
#include <cstdint>
#include <cstdio>

namespace {

constexpr const char *recordingPath = "/usd/recording.csv";
constexpr int samplePeriodMs = 1;

std::atomic<bool> recordingRequested{false};
std::atomic<bool> recordingActive{false};
pros::Task *recordingTask = nullptr;

void recordLoop() {
    FILE *file = nullptr;
    std::uint32_t startTime = 0;

    while (true) {
        if (recordingRequested.load() && !file) {
            if (pros::usd::is_installed()) {
                file = std::fopen(recordingPath, "w");
                if (file) {
                    std::fprintf(
                        file,
                        "timestamp_ms,x_in,y_in,heading_rad\n"
                    );
                    std::fflush(file);
                    startTime = pros::millis();
                    recordingActive.store(true);
                } else {
                    recordingRequested.store(false);
                }
            } else {
                recordingRequested.store(false);
            }
        }

        if (!recordingRequested.load() && file) {
            std::fclose(file);
            file = nullptr;
            recordingActive.store(false);
        }

        if (file) {
            std::fprintf(
                file,
                "%u,%.4f,%.4f,%.6f\n",
                pros::millis() - startTime,
                static_cast<double>(globalXIn.load()),
                static_cast<double>(globalYIn.load()),
                static_cast<double>(globalHeading.load())
            );
            std::fflush(file);
        }

        pros::delay(samplePeriodMs);
    }
}

} // namespace

namespace recorder {

bool is_recording() {
    return recordingActive.load();
}

bool is_requested() {
    return recordingRequested.load();
}

bool start() {
    if (!pros::usd::is_installed()) return false;
    recordingRequested.store(true);
    return true;
}

void stop() {
    recordingRequested.store(false);
}

void toggle() {
    if (recordingRequested.load()) {
        stop();
    } else {
        start();
    }
}

void create_button(lv_obj_t *parent) {
    lv_obj_t *panel = lv_obj_create(parent);
    lv_obj_set_size(panel, 196, 70);
    lv_obj_align(panel, LV_ALIGN_BOTTOM_RIGHT, -8, -8);
    lv_obj_clear_flag(panel, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *title = lv_label_create(panel);
    lv_label_set_text(title, "Driver recorder");
    lv_obj_align(title, LV_ALIGN_TOP_LEFT, 8, 2);

    lv_obj_t *button = lv_btn_create(panel);
    lv_obj_set_size(button, 178, 34);
    lv_obj_align(button, LV_ALIGN_BOTTOM_MID, 0, -4);

    lv_obj_set_style_bg_color(button, lv_color_make(55, 115, 255), LV_PART_MAIN);
    lv_obj_set_style_bg_color(button, lv_color_make(255, 50, 50), LV_PART_MAIN | LV_STATE_CHECKED);
    lv_obj_set_style_text_color(button, lv_color_make(255, 255, 255), LV_PART_MAIN);
    lv_obj_set_style_border_opa(button, LV_OPA_0, 0);
    lv_obj_set_style_radius(button, 8, 0);

    lv_obj_t *label = lv_label_create(button);
    lv_label_set_text(label, "START RECORDING");
    lv_obj_center(label);
    lv_obj_set_user_data(button, label);
    lv_obj_add_event_cb(
        button,
        [](lv_event_t *event) {
            lv_obj_t *target = lv_event_get_target(event);
            lv_obj_t *recordingLabel =
                static_cast<lv_obj_t *>(lv_obj_get_user_data(target));

            const bool wasRecording = recorder::is_requested();
            if (wasRecording) {
                recorder::stop();
            } else {
                if (!recorder::start()) {
                    return;
                }
            }

            const bool nowRecording = recorder::is_requested();
            lv_label_set_text(
                recordingLabel,
                nowRecording ? "STOP RECORDING" : "START RECORDING"
            );

            if (nowRecording) {
                lv_obj_add_state(target, LV_STATE_CHECKED);
            } else {
                lv_obj_clear_state(target, LV_STATE_CHECKED);
            }
        },
        LV_EVENT_CLICKED,
        nullptr
    );
}

} // namespace recorder

void initializeRecorderTask() {
    if (!recordingTask) recordingTask = new pros::Task(recordLoop, "Recorder");
}
