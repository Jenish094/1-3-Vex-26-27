#include "robot_hardware.hpp"

#include "pros/misc.h"
#include "pros/misc.hpp"

Hardware devices;

void MWrap::set(int speed)
{
        if (motor) {
                speed == 0 ? motor->brake() : motor->move(speed);
        } else {
                MWrap_Init();
        }
}

MWrap::~MWrap(void) = default;

MGWrap::MGWrap(MWrap* first, MWrap* second, MWrap* third, float scale)
        : powerScale(scale)
{
        motors.reserve(3);
        motors.insert(motors.end(), {first, second, third});
}

void Hardware::LoadPeripherals(void)
{
        MGroups.DT_L = {&Motors.MOTOR_L1, &Motors.MOTOR_L2, nullptr};
        MGroups.DT_R = {&Motors.MOTOR_R1, &Motors.MOTOR_R2, nullptr, 0.97f};
        MGroups.MOTORGROUP_INTAKE = {&Motors.MOTOR_INTAKE_11W, nullptr, nullptr};

        Pneumatics.PNEUMATIC = std::make_unique<pros::adi::Pneumatics>('B', false);
        PRIMARY_CONTROLLER = std::make_unique<pros::Controller>(
                pros::controller_id_e_t::E_CONTROLLER_MASTER);
        SECONDARY_CONTROLLER = std::make_unique<pros::Controller>(
                pros::controller_id_e_t::E_CONTROLLER_PARTNER);

        Odom.VerticalWheel = std::make_unique<pros::Rotation>(odom_vertical);
        Odom.HorizontalWheel = std::make_unique<pros::Rotation>(odom_horizontal);
        Sensors.IMU = std::make_unique<pros::Imu>(imu);
        Sensors.CascadeEncoder = std::make_unique<pros::Rotation>(cascade_encoder);
        Sensors.ToggleOptical = std::make_unique<pros::Optical>(optical);
        Macros.flipToggleHeight = std::make_unique<FlipToggleHeight>();
}

Hardware::Hardware(void)
        : Motors{
                  MWrap(motor_left_front, pros::E_MOTOR_GEAR_BLUE),
                  MWrap(motor_left_rear, pros::E_MOTOR_GEAR_BLUE),
                  MWrap(motor_right_front, pros::E_MOTOR_GEAR_BLUE),
                  MWrap(motor_right_rear, pros::E_MOTOR_GEAR_BLUE),
                  MWrap(motor_intake_11W, pros::E_MOTOR_GEAR_GREEN),
                  MWrap(motor_cascade_11WR, pros::E_MOTOR_GEAR_GREEN),
                  MWrap(motor_cascade_11WL, pros::E_MOTOR_GEAR_GREEN),
                  MWrap(motor_score_open, pros::E_MOTOR_GEAR_GREEN),
                  MWrap(motor_score_roll, pros::E_MOTOR_GEAR_GREEN)}
{
}

void Hardware::init(void)
{
        LoadPeripherals();

        for (MWrap* motor : {&Motors.MOTOR_L1, &Motors.MOTOR_L2,
                             &Motors.MOTOR_R1, &Motors.MOTOR_R2,
                             &Motors.MOTOR_CASCADE_11WR,
                             &Motors.MOTOR_CASCADE_11WL,
                             &Motors.MOTOR_INTAKE_11W,
                             &Motors.MOTOR_SCORE_OPEN,
                             &Motors.MOTOR_SCORE_ROLL}) {
                motor->MWrap_Init();
        }

        Odom.VerticalWheel->set_reversed(true);
        Odom.VerticalWheel->reset_position();
        Odom.VerticalWheel->reset();
        Odom.HorizontalWheel->set_reversed(false);
        Odom.HorizontalWheel->reset_position();
        Odom.HorizontalWheel->reset();

        Sensors.CascadeEncoder->set_reversed(false);
        Sensors.CascadeEncoder->reset_position();
        Sensors.ToggleOptical->set_led_pwm(100);
        Sensors.IMU->reset(true);

        globalXIn = 0.0f;
        globalYIn = 0.0f;
        globalHeading = 0.0f;
}

Hardware::~Hardware(void) = default;
