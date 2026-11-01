#ifndef ROBOT_HARDWARE_HPP
#define ROBOT_HARDWARE_HPP

#include "api.h"
#include "macro.hpp"
#include "odom.hpp"

#include <cstdint>
#include <memory>
#include <vector>

using pros_adi_port = uint8_t;
using pros_standard_port = int8_t;

class FlipToggleHeight;

// negative port is reversed
enum StandardPorts : pros_standard_port
{
        LEFT_FRONT = -14,
        LEFT_REAR = -15,
        RIGHT_FRONT = 11,
        RIGHT_REAR = 12,

        IMU_PORT = 5,
        AI_VISION_PORT = 9,
        INTAKE_PORT = -16,
        CASCADE_RIGHT_PORT = -19,
        CASCADE_LEFT_PORT = 20,
        SCORE_OPEN_PORT = -1,
        SCORE_ROLL_PORT = -2,

        VERTICAL_ODOM_PORT = 3,
        HORIZONTAL_ODOM_PORT = 4,
        CASCADE_ENCODER_PORT = 6,
        OPTICAL_PORT = 8
};
constexpr pros_standard_port motor_left_front = LEFT_FRONT;
constexpr pros_standard_port motor_left_rear = LEFT_REAR;
constexpr pros_standard_port motor_right_front = RIGHT_FRONT;
constexpr pros_standard_port motor_right_rear = RIGHT_REAR;
constexpr pros_standard_port imu = IMU_PORT;
constexpr pros_standard_port aivision = AI_VISION_PORT;
constexpr pros_standard_port motor_intake_11W = INTAKE_PORT;
constexpr pros_standard_port motor_cascade_11WR = CASCADE_RIGHT_PORT;
constexpr pros_standard_port motor_cascade_11WL = CASCADE_LEFT_PORT;
constexpr pros_standard_port motor_score_open = SCORE_OPEN_PORT;
constexpr pros_standard_port motor_score_roll = SCORE_ROLL_PORT;
constexpr pros_standard_port odom_vertical = VERTICAL_ODOM_PORT;
constexpr pros_standard_port odom_horizontal = HORIZONTAL_ODOM_PORT;
constexpr pros_standard_port cascade_encoder = CASCADE_ENCODER_PORT;
constexpr pros_standard_port optical = OPTICAL_PORT;

enum ThreeWire
{
};

struct MWrap
{
        std::unique_ptr<pros::Motor> motor;
        pros_standard_port port;
        pros::motor_gearset_e_t gearset;

        MWrap()
                : motor(nullptr),
                  port(0),
                  gearset(pros::E_MOTOR_GEAR_BLUE)
        {
        }

        explicit MWrap(pros_standard_port motorPort,
                       pros::motor_gearset_e_t motorGear = pros::E_MOTOR_GEAR_BLUE)
                : motor(nullptr),
                  port(motorPort),
                  gearset(motorGear)
        {
        }

        ~MWrap(void);

        void MWrap_Init()
        {
                if (motor) {
                        return;
                }

                motor = std::make_unique<pros::Motor>(port);
                motor->set_brake_mode(pros::MotorBrake::brake);
                motor->set_gearing(gearset);
        }

        void operator=(pros::Motor motorValue)
        {
                motor = std::make_unique<pros::Motor>(motorValue);
        }

        void set(int speed);
};

struct MGWrap
{
        std::vector<MWrap*> motors;
        float powerScale;

        MGWrap()
                : powerScale(1.0f)
        {
        }

        MGWrap(MWrap* first, MWrap* second, MWrap* third,
               float scale = 1.0f);

        void set(int speed)
        {
                const int output = static_cast<int>(speed * powerScale);
                for (MWrap* motor : motors) {
                        if (motor != nullptr) {
                                motor->set(output);
                        }
                }
        }
};

struct Hardware
{
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

        struct
        {
                std::unique_ptr<FlipToggleHeight> flipToggleHeight;
        } Macros;

        std::unique_ptr<pros::Controller> PRIMARY_CONTROLLER;
        std::unique_ptr<pros::Controller> SECONDARY_CONTROLLER;

        Hardware(void);
        ~Hardware(void);

        void LoadPeripherals(void);
        void init(void);

        Hardware(const Hardware&) = delete;
        Hardware& operator=(const Hardware&) = delete;
};

extern Hardware devices;

#endif
