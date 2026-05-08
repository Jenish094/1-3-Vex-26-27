#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep

pros::Controller controller(pros::E_CONTROLLER_MASTER);
pros::MotorGroup leftdt({-1, -2}, pros::MotorGearset::green);
pros::MotorGroup rightdt({-3, -4}, pros::MotorGearset::green);

lemlib::Drivetrain drivetrain(&leftdt,
							  &rightdt, 
								  16.5,
								  lemlib::Omniwheel::NEW_275,
							  360,
							  2 // horizontal drift need to recalibrate
							  );

pros::Imu imu(10); //IMU sensor port 10
//tracking wheels
//horizontal tracking wheel's sensor port 20, not reversed
pros::Rotation horizontalEnc('20'); // perpendicular to drivetrain
pros::Rotation verticalEnc('11'); // paralell to drivetrain
// horizontal tracking wheel size default is 2 inch with wheel 2 inches right to the IMU center
lemlib::TrackingWheel horizontal(&horizontalEnc, lemlib::Omniwheel::NEW_2, 2);
// vertical tracking wheel size default is 2.75 inch with wheel 1.7 inches right to the IMU center
lemlib::TrackingWheel vertical(&verticalEnc, lemlib::Omniwheel::NEW_2, 1.7);

lemlib::OdomSensors sensors(&vertical, nullptr, &horizontal, nullptr, &imu);
// odomsensors, 1 vertical wheel, 1 horizontal, replace nullptr with second tracking wheels if needed

// lateral PID controller
lemlib::ControllerSettings lateral_controller(10, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              3, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              20 // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angular_controller(2, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              10, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in degrees
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in degrees
                                              500, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

// input curve for throttle input during driver control
lemlib::ExpoDriveCurve throttle_curve(3, // joystick deadband out of 127
                                     10, // minimum output where drivetrain will move out of 127
                                     1.019 // expo curve gain
);

// input curve for steer input during driver control
lemlib::ExpoDriveCurve steer_curve(3, // joystick deadband out of 127
                                  10, // minimum output where drivetrain will move out of 127
                                  1.019 // expo curve gain
);

lemlib::Chassis chassis(drivetrain, lateral_controller, angular_controller, sensors, &throttle_curve, &steer_curve);
