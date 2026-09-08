#include "hardware.hpp"

// Define the global instance here
Hardware devices; 

void MotorWrapper::set(int speed) {
    if (motor)
        speed != 0 ? motor->move(speed) : motor->brake();
    else
        MotorWrapper_Init();
}

MotorWrapper::~MotorWrapper(void) {}

MotorGroupWrapper::MotorGroupWrapper(MotorWrapper* m1, MotorWrapper* m2, MotorWrapper* m3, float powerScale) {
    this->powerScale = powerScale;
    this->motors.reserve(3);
    this->motors.push_back(m1);
    this->motors.push_back(m2);
    this->motors.push_back(m3);
}

void Hardware::LoadPeripherals(void) {
    this->MGroups.DT_L = MotorGroupWrapper(&Motors.MOTOR_L1, &Motors.MOTOR_L2, nullptr);
    this->MGroups.DT_R = MotorGroupWrapper(&Motors.MOTOR_R1, &Motors.MOTOR_R2, nullptr, 0.97f);
    this->MGroups.MOTORGROUP_INTAKE = MotorGroupWrapper(&Motors.MOTOR_INTAKE_11W, nullptr, nullptr);
    
    this->Pneumatics.PNEUMATIC = std::make_unique<pros::adi::Pneumatics>('B', false);

    this->PRIMARY_CONTROLLER = std::make_unique<pros::Controller>(pros::controller_id_e_t::E_CONTROLLER_MASTER);
    this->Odom.VerticalWheel = std::make_unique<pros::Rotation>(odom_vertical);
    this->Odom.HorizontalWheel = std::make_unique<pros::Rotation>(odom_horizontal);
    this->Sensors.IMU = std::make_unique<pros::Imu>(imu);

    this->Macros.flipToggleHeight = std::make_unique<FlipToggleHeight>();
}

// REMOVE ALL HARDWARE INTERACTION FROM THE CONSTRUCTOR
Hardware::Hardware(void) : Motors{ 
    MotorWrapper(motor_left_front, pros::E_MOTOR_GEAR_BLUE), 
    MotorWrapper(motor_left_rear, pros::E_MOTOR_GEAR_BLUE), 
    MotorWrapper(motor_right_front, pros::E_MOTOR_GEAR_BLUE), 
    MotorWrapper(motor_right_rear, pros::E_MOTOR_GEAR_BLUE), 
    MotorWrapper(motor_intake_11W, pros::E_MOTOR_GEAR_GREEN), 
    MotorWrapper(motor_cascade_11WR, pros::E_MOTOR_GEAR_GREEN), 
    MotorWrapper(motor_cascade_11WL, pros::E_MOTOR_GEAR_GREEN), 
    MotorWrapper(motor_score_open, pros::E_MOTOR_GEAR_GREEN), 
    MotorWrapper(motor_score_roll, pros::E_MOTOR_GEAR_GREEN) } 
{
    // Leave this completely blank or just structural assignments.
    // Do not call LoadPeripherals() here!
}

// Create a safe initialization function to run inside main.cpp's initialize()
void Hardware::init(void) {
    LoadPeripherals();
    
    this->Motors.MOTOR_L1.MotorWrapper_Init();
    this->Motors.MOTOR_L2.MotorWrapper_Init();
    this->Motors.MOTOR_R1.MotorWrapper_Init();
    this->Motors.MOTOR_R2.MotorWrapper_Init();

    this->Motors.MOTOR_CASCADE_11WR.MotorWrapper_Init();
    this->Motors.MOTOR_CASCADE_11WL.MotorWrapper_Init();
    this->Motors.MOTOR_INTAKE_11W.MotorWrapper_Init();
    this->Motors.MOTOR_SCORE_OPEN.MotorWrapper_Init();
    this->Motors.MOTOR_SCORE_ROLL.MotorWrapper_Init();

    this->Odom.VerticalWheel->set_reversed(true);
    this->Odom.VerticalWheel->reset_position();
    this->Odom.VerticalWheel->reset();

    this->Odom.HorizontalWheel->set_reversed(false);
    this->Odom.HorizontalWheel->reset_position();
    this->Odom.HorizontalWheel->reset();

    this->Sensors.IMU->reset(true); // Now safe to run blockingly!

    globalXIn = 0.0f; globalYIn = 0.0f; globalHeading = 0.0f;
}

Hardware::~Hardware(void) {}
