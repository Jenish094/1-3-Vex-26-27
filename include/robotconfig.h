#include "lemlib/api.hpp" // IWYU pragma: keep
#include "pros/rotation.hpp"

// Robot configuration declarations
extern pros::Controller controller;
extern pros::MotorGroup leftdt;
extern pros::MotorGroup rightdt;
extern lemlib::Drivetrain drivetrain;
extern pros::Imu imu;
extern pros::Rotation horizontalEnc;
extern pros::Rotation verticalEnc;
extern lemlib::TrackingWheel horizontal;
extern lemlib::TrackingWheel vertical;
extern lemlib::OdomSensors sensors;
extern lemlib::ControllerSettings lateral_controller;
extern lemlib::ControllerSettings angular_controller;
extern lemlib::ExpoDriveCurve throttle_curve;
extern lemlib::ExpoDriveCurve steer_curve; 
extern lemlib::Chassis chassis;
extern pros::Motor Lift;
extern pros::adi::DigitalOut Claw;