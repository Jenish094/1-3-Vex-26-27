#pragma once
#include "api.h"
#include "helpers.hpp"
#include "odom.hpp"
#include "vex_macro.hpp"
#include <memory>

class FlipToggleHeight;

enum StandardPorts
{ // Negative == reversed
        motor_left_front = -14,
        motor_left_rear = -15,

        motor_right_front = 11,
        motor_right_rear = 12,

        imu = 5,
        aivision = 6,

        motor_intake_11W = -16,

        motor_cascade_11WR = -19,
        motor_cascade_11WL = 20,

        motor_score_open = -1,
        motor_score_roll = -2,

        odom_vertical = 3,
        odom_horizontal = 4,

        cascade_encoder = 6,

        distance = 7
};

enum Triports
{

};

struct MotorWrapper
{
        std::unique_ptr<pros::Motor> motor;
        pros_standard_port port;
        pros::motor_gearset_e_t gearset;

        MotorWrapper(void) : port(0), gearset(pros::E_MOTOR_GEAR_BLUE) {};
        MotorWrapper(const pros_standard_port port, const pros::motor_gearset_e_t gearset = pros::E_MOTOR_GEAR_BLUE) : port(port), gearset(gearset) {};
        ~MotorWrapper(void);

        void MotorWrapper_Init() {
                if (!motor) {
                        motor = std::make_unique<pros::Motor>(port);
                        motor->set_brake_mode(pros::MotorBrake::brake);
                        motor->set_gearing(gearset);
                }
        }

        void operator=(pros::Motor m) {
                this->motor = std::make_unique<pros::Motor>(m);
        }

        void set(int speed);
};

struct MotorGroupWrapper
{
        std::vector<MotorWrapper*> motors;
        float powerScale = 1.0f;

        void set(int speed) {
                int scaledSpeed = static_cast<int>(speed * powerScale);
                for (MotorWrapper* m : motors) {
                        if (m)
                                m->set(scaledSpeed);
                }
        }

        MotorGroupWrapper(void) {};
        MotorGroupWrapper(MotorWrapper* m1, MotorWrapper* m2, MotorWrapper* m3, float powerScale = 1.0f);
};

struct Hardware
{
public:
        struct
        {
                MotorWrapper MOTOR_L1;
                MotorWrapper MOTOR_L2;

                MotorWrapper MOTOR_R1;
                MotorWrapper MOTOR_R2;

                MotorWrapper MOTOR_INTAKE_11W;
                MotorWrapper MOTOR_CASCADE_11WR;
                MotorWrapper MOTOR_CASCADE_11WL;
                MotorWrapper MOTOR_SCORE_OPEN;
                MotorWrapper MOTOR_SCORE_ROLL;
        } Motors;

        struct
        {
                MotorGroupWrapper DT_L;
                MotorGroupWrapper DT_R;
                MotorGroupWrapper MOTORGROUP_INTAKE;
        } MGroups;

        std::unique_ptr<pros::Controller> PRIMARY_CONTROLLER;
        std::unique_ptr<pros::Controller> SECONDARY_CONTROLLER;

        struct
        {
                std::unique_ptr<pros::adi::Pneumatics> PNEUMATIC;
        } Pneumatics;

        struct
        {
                std::unique_ptr<pros::Rotation> VerticalWheel;
                std::unique_ptr<pros::Rotation> HorizontalWheel;
        } Odom;

        struct
        {
                std::unique_ptr<pros::Imu> IMU;
                std::unique_ptr<pros::Rotation> CascadeEncoder;
        } Sensors;

        struct {
                std::unique_ptr<FlipToggleHeight> flipToggleHeight;
        } Macros;

        Hardware(void);
        ~Hardware(void);

        void LoadPeripherals(void);
        void init(void);


private:
        Hardware(const Hardware&) = delete;
        Hardware& operator=(const Hardware&) = delete;
};

extern Hardware devices;
