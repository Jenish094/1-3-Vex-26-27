#include "driver.hpp"

std::atomic<bool> buttonAState(false);
std::atomic<bool> buttonBState(false);
std::atomic<bool> buttonXState(false);
std::atomic<bool> buttonYState(false);
std::atomic<bool> buttonUpState(false);
std::atomic<bool> buttonDownState(false);
std::atomic<bool> buttonLeftState(false);
std::atomic<bool> buttonRightState(false);

std::atomic<bool> bumperLeftFrontState(false);
std::atomic<bool> bumperLeftBackState(false);
std::atomic<bool> bumperRightFrontState(false);
std::atomic<bool> bumperRightBackState(false);

std::atomic<int> axisLeftX(0);
std::atomic<int> axisLeftY(0);
std::atomic<int> axisRightX(0);
std::atomic<int> axisRightY(0);

void driverReadInput(void) {
        do {
                buttonAState = devices.PRIMARY_CONTROLLER->get_digital(pros::controller_digital_e_t::E_CONTROLLER_DIGITAL_A);
                buttonBState = devices.PRIMARY_CONTROLLER->get_digital(pros::controller_digital_e_t::E_CONTROLLER_DIGITAL_B);
                buttonXState = devices.PRIMARY_CONTROLLER->get_digital(pros::controller_digital_e_t::E_CONTROLLER_DIGITAL_X);
                buttonYState = devices.PRIMARY_CONTROLLER->get_digital(pros::controller_digital_e_t::E_CONTROLLER_DIGITAL_Y);
                buttonUpState = devices.PRIMARY_CONTROLLER->get_digital(pros::controller_digital_e_t::E_CONTROLLER_DIGITAL_UP);
                buttonDownState = devices.PRIMARY_CONTROLLER->get_digital(pros::controller_digital_e_t::E_CONTROLLER_DIGITAL_DOWN);
                buttonLeftState = devices.PRIMARY_CONTROLLER->get_digital(pros::controller_digital_e_t::E_CONTROLLER_DIGITAL_LEFT);
                buttonRightState = devices.PRIMARY_CONTROLLER->get_digital(pros::controller_digital_e_t::E_CONTROLLER_DIGITAL_RIGHT);


                axisLeftX = devices.PRIMARY_CONTROLLER->get_analog(pros::controller_analog_e_t::E_CONTROLLER_ANALOG_LEFT_X);
                axisLeftY = devices.PRIMARY_CONTROLLER->get_analog(pros::controller_analog_e_t::E_CONTROLLER_ANALOG_LEFT_Y);
                axisRightX = devices.PRIMARY_CONTROLLER->get_analog(pros::controller_analog_e_t::E_CONTROLLER_ANALOG_RIGHT_X);
                axisRightY = devices.PRIMARY_CONTROLLER->get_analog(pros::controller_analog_e_t::E_CONTROLLER_ANALOG_RIGHT_Y);

                bumperLeftFrontState = devices.PRIMARY_CONTROLLER->get_digital(pros::controller_digital_e_t::E_CONTROLLER_DIGITAL_L1);
                bumperLeftBackState = devices.PRIMARY_CONTROLLER->get_digital(pros::controller_digital_e_t::E_CONTROLLER_DIGITAL_L2);
                bumperRightFrontState = devices.PRIMARY_CONTROLLER->get_digital(pros::controller_digital_e_t::E_CONTROLLER_DIGITAL_R1);
                bumperRightBackState = devices.PRIMARY_CONTROLLER->get_digital(pros::controller_digital_e_t::E_CONTROLLER_DIGITAL_R2);

                pros::delay(10); // could even be lower?
        } while (true);
}

void driverApplyInput(void) {
        do {
                if (!isAuton)
                {
                        devices.MGroups.DT_L.set(axisLeftY.load() + axisRightX.load());
                        devices.MGroups.DT_R.set(axisLeftY.load() - axisRightX.load());
                }
                pros::delay(9);
        } while (true);
}


void driverScreenManager(void) {
        do {
                devices.PRIMARY_CONTROLLER->clear();
                devices.PRIMARY_CONTROLLER->print(0, 0, "Robot power: %d% (%iV/%iA)", pros::battery::get_capacity(), pros::battery::get_voltage(), pros::battery::get_current());

                devices.PRIMARY_CONTROLLER->print(2, 0, "Drivetrain temperature:");

                if (isAnyDriveMotorTempAbove(70.0f))
                        devices.PRIMARY_CONTROLLER->print(2, 25, "CRITICAL");
                else if (isAnyDriveMotorTempAbove(55.0f))
                        devices.PRIMARY_CONTROLLER->print(2, 25, "OVERHEAT");
                else
                        devices.PRIMARY_CONTROLLER->print(2, 25, "SAFE");

                devices.PRIMARY_CONTROLLER->print(2, 0, "SPEED (KMPH):");
                devices.PRIMARY_CONTROLLER->print(3, 0, "%f", calcSpeedKmPerH());

                pros::delay(500);
        } while (true);
}


bool isAnyMotorTempAbove(float temp) {
        if (isAnyDriveMotorTempAbove(temp))
                return true;

        for (MotorWrapper* m : devices.MGroups.MOTORGROUP_INTAKE.motors) { // TODO: fix
                if (m->motor->get_temperature() >= temp)
                        return true;
        }

        return false;
}

bool isAnyDriveMotorTempAbove(float temp) {
        for (MotorWrapper* m : devices.MGroups.DT_L.motors)
                if (m->motor->get_temperature() >= temp)
                        return true;

        for (MotorWrapper* m : devices.MGroups.DT_R.motors)
                if (m->motor->get_temperature() >= temp)
                        return true;

        return false;
}

int calcSpeedKmPerH(void) {
        int timestepMs = 5;

        Vector2 lastPos = Vector2(lastGlobalXIn, lastGlobalYIn);
        Vector2 thisPos = Vector2(globalXIn, globalYIn);

        float distIn = lastPos.Distance(thisPos);
        float distCm = distIn * 2.54f;

        float distKm = distCm * 10e5;

        float kmPerS = distKm / (timestepMs / 1000.0f);
        float kmPerH = kmPerS / 3600.0f;

        return kmPerH;
}