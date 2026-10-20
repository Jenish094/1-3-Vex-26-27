#include "hardware.hpp"
#include "pros/misc.h"
#include "pros/misc.hpp"
#include <memory>

// Define the global instance here
Hardware devices; 

void MWrap::set(int speed) {
    if (motor)
        speed != 0 ? motor->move(speed) : motor->brake();
    else
        MWrap_Init();
}

MWrap::~MWrap(void) {}

MGWrap::MGWrap(MWrap* m1, MWrap* m2, MWrap* m3, float powerScale) {
    this->powerScale = powerScale;
    this->motors.reserve(3);
    this->motors.push_back(m1);
    this->motors.push_back(m2);
    this->motors.push_back(m3);
}

void Hardware::LoadPeripherals(void) {
    this->MGroups.DT_L = MGWrap(&Motors.MOTOR_L1, &Motors.MOTOR_L2, nullptr);
    this->MGroups.DT_R = MGWrap(&Motors.MOTOR_R1, &Motors.MOTOR_R2, nullptr, 0.97f);
    this->MGroups.MOTORGROUP_INTAKE = MGWrap(&Motors.MOTOR_INTAKE_11W, nullptr, nullptr);
    
    this->Pneumatics.PNEUMATIC = std::make_unique<pros::adi::Pneumatics>('B', false);

    this->PRIMARY_CONTROLLER = std::make_unique<pros::Controller>(pros::controller_id_e_t::E_CONTROLLER_MASTER);
    this->SECONDARY_CONTROLLER = std::make_unique<pros::Controller>(pros::controller_id_e_t::E_CONTROLLER_PARTNER);
    this->Odom.VerticalWheel = std::make_unique<pros::Rotation>(odom_vertical);
    this->Odom.HorizontalWheel = std::make_unique<pros::Rotation>(odom_horizontal);
    this->Sensors.IMU = std::make_unique<pros::Imu>(imu);
    this->Sensors.CascadeEncoder = std::make_unique<pros::Rotation>(cascade_encoder);
    this->Sensors.ToggleOptical = std::make_unique<pros::Optical>(optical);

    this->Macros.flipToggleHeight = std::make_unique<FlipToggleHeight>();
}

// REMOVE ALL HARDWARE INTERACTION FROM THE CONSTRUCTOR
Hardware::Hardware(void) : Motors{ 
    MWrap(motor_left_front, pros::E_MOTOR_GEAR_BLUE), 
    MWrap(motor_left_rear, pros::E_MOTOR_GEAR_BLUE), 
    MWrap(motor_right_front, pros::E_MOTOR_GEAR_BLUE), 
    MWrap(motor_right_rear, pros::E_MOTOR_GEAR_BLUE), 
    MWrap(motor_intake_11W, pros::E_MOTOR_GEAR_GREEN), 
    MWrap(motor_cascade_11WR, pros::E_MOTOR_GEAR_GREEN), 
    MWrap(motor_cascade_11WL, pros::E_MOTOR_GEAR_GREEN), 
    MWrap(motor_score_open, pros::E_MOTOR_GEAR_GREEN), 
    MWrap(motor_score_roll, pros::E_MOTOR_GEAR_GREEN) } 
{
    // Leave this completely blank or just structural assignments.
}

// Create a safe initialization function to run inside main.cpp's initialize()
void Hardware::init(void) {
    LoadPeripherals();
    
    this->Motors.MOTOR_L1.MWrap_Init();
    this->Motors.MOTOR_L2.MWrap_Init();
    this->Motors.MOTOR_R1.MWrap_Init();
    this->Motors.MOTOR_R2.MWrap_Init();

    this->Motors.MOTOR_CASCADE_11WR.MWrap_Init();
    this->Motors.MOTOR_CASCADE_11WL.MWrap_Init();
    this->Motors.MOTOR_INTAKE_11W.MWrap_Init();
    this->Motors.MOTOR_SCORE_OPEN.MWrap_Init();
    this->Motors.MOTOR_SCORE_ROLL.MWrap_Init();

    this->Odom.VerticalWheel->set_reversed(true);
    this->Odom.VerticalWheel->reset_position();
    this->Odom.VerticalWheel->reset();

    this->Odom.HorizontalWheel->set_reversed(false);
    this->Odom.HorizontalWheel->reset_position();
    this->Odom.HorizontalWheel->reset();

    this->Sensors.CascadeEncoder->set_reversed(false);
    this->Sensors.CascadeEncoder->reset_position();
    
    this->Sensors.ToggleOptical->set_led_pwm(100);

    this->Sensors.IMU->reset(true); // Now safe to run blockingly!

    globalXIn = 0.0f; globalYIn = 0.0f; globalHeading = 0.0f;
}

Hardware::~Hardware(void) {}
