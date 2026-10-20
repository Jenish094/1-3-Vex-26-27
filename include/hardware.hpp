#ifndef HARDWARE_HPP
#define HARDWARE_HPP

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
        aivision = 9,

        motor_intake_11W = -16,

        motor_cascade_11WR = -19,
        motor_cascade_11WL = 20,

        motor_score_open = -1,
        motor_score_roll = -2,

        odom_vertical = 3,
        odom_horizontal = 4,

        cascade_encoder = 6,

        distance = 7,

        optical = 8
};

enum Triports
{

};

struct MWrap
{
        std::unique_ptr<pros::Motor> motor;
        pros_standard_port port;
        pros::motor_gearset_e_t gearset;

        MWrap(void) : port(0), gearset(pros::E_MOTOR_GEAR_BLUE) {};
        MWrap(const pros_standard_port port, const pros::motor_gearset_e_t gearset = pros::E_MOTOR_GEAR_BLUE) : port(port), gearset(gearset) {};
        ~MWrap(void);

        void MWrap_Init() {
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

struct MGWrap
{
        std::vector<MWrap*> motors;
        float powerScale = 1.0f;

        void set(int speed) {
                int scaledSpeed = static_cast<int>(speed * powerScale);
                for (MWrap* m : motors) {
                        if (m)
                                m->set(scaledSpeed);
                }
        }

        MGWrap(void) {};
        MGWrap(MWrap* m1, MWrap* m2, MWrap* m3, float powerScale = 1.0f);
};

struct Hardware
{
public:
        struct
        {
                MWrap MOTOR_L1;
                MWrap MOTOR_L2;

                MWrap MOTOR_R1;
                MWrap MOTOR_R2;

                MWrap MOTOR_INTAKE_11W;
                MWrap MOTOR_CASCADE_11WR;
                MWrap MOTOR_CASCADE_11WL;
                MWrap MOTOR_SCORE_OPEN;
                MWrap MOTOR_SCORE_ROLL;
        } Motors;

        struct
        {
                MGWrap DT_L;
                MGWrap DT_R;
                MGWrap MOTORGROUP_INTAKE;
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
                std::unique_ptr<pros::Optical> ToggleOptical;
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

#endif
