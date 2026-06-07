#include "auto.h"
#include "robotconfig.h"
#include "lemlib/api.hpp" // IWYU pragma: keep

ASSET(redfront_txt);

void redFront() {
    chassis.setPose(0, 0, 0);
    chassis.follow(redfront_txt, 15, 2000);

}

void redLeft() {
}

void blueFront() {
}

void blueLeft() {
}

void skills_auton() {
}
