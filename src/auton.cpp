// #include "auton.hpp"
// #include "hardware.hpp"
// #include <cmath>
// #include <iostream>

// // Setup global tracking variables from your hardware configuration
// // Change these to your actual tuning values or external globals if defined elsewhere
// float turnKp = 3.5f, turnKi = 0.01f, turnKd = 10.0f;
// float driveKp = 2.5f, driveKi = 0.005f, driveKd = 8.0f;
// float turnOutputPowerMod = 1.0f;

// // Wheel scaling conversion math
// const double WHEEL_DIAMETER = 2.75; 
// const double TICKS_PER_ROTATION = 360.0; 
// const double INCH_TO_TICKS = TICKS_PER_ROTATION / (WHEEL_DIAMETER * M_PI);

// // Quick helper clamp function to avoid missing dependencies
// float Clamp(float value, float min, float max) {
//     if (value < min) return min;
//     if (value > max) return max;
//     return value;
// }

// void auton_faceHeading(float targetHeading, int timeout_millis) {
//         int startTimeMillis = pros::millis();
        
//         // 1. Grab current orientation from the physical Inertial IMU sensor
//         float currentHeading = devices.Sensors.IMU.get_heading();
//         float error = targetHeading - currentHeading;
        
//         // Handle compass wrap-around logic (-180 to 180 degrees code layout)
//         while (error > 180.0f) error -= 360.0f;
//         while (error < -180.0f) error += 360.0f;

//         int dT = 10;
//         float turnIntegral = 0.0f;
//         float turnPastHeading = currentHeading;
//         float PID = 10.0f;

//         while ((std::abs(error) >= 1.0f || std::abs(PID) >= 5.0f) && (pros::millis() - startTimeMillis) <= timeout_millis) {
//                 currentHeading = devices.Sensors.IMU.get_heading();
                
//                 error = targetHeading - currentHeading;
//                 while (error > 180.0f) error -= 360.0f;
//                 while (error < -180.0f) error += 360.0f;

//                 float prop = error * turnKp;
//                 prop = Clamp(prop, -127.0f, 127.0f);

//                 float deriv = turnKd * (turnPastHeading - currentHeading);

//                 turnIntegral *= 0.95f; 
//                 turnIntegral += error;
//                 turnIntegral = Clamp(turnIntegral, -1000.0f, 1000.0f);
//                 float integ = turnKi * turnIntegral;

//                 PID = prop + integ + deriv;
//                 PID = Clamp(PID, -127.0f, 127.0f);

//                 turnPastHeading = currentHeading;

//                 devices.MGroups.DT_L.set(PID * turnOutputPowerMod);
//                 devices.MGroups.DT_R.set(-PID * turnOutputPowerMod);

//                 pros::delay(dT);
//         }
//         devices.MGroups.DT_L.set(0);
//         devices.MGroups.DT_R.set(0);
// }

// void auton_driveDistance(int targetInches, int timeout_millis) {
//         // 2. Reset your single physical vertical tracking wheel to 0 before driving
//         devices.Sensors.ENCODER.reset();
        
//         float targetTicks = targetInches * INCH_TO_TICKS;
//         float currentTicks = devices.Sensors.ENCODER.get_value();
//         float error = targetTicks - currentTicks;

//         float startTimeMillis = pros::millis();
//         int dT = 10;
//         float driveIntegral = 0.0f;
//         float drivePastTicks = currentTicks;
//         float PID = 10.0f;

//         while ((std::abs(error) >= (0.25f * INCH_TO_TICKS) || std::abs(PID) >= 5.0f) && (pros::millis() - startTimeMillis) <= timeout_millis) {
//                 currentTicks = devices.Sensors.IMU->get_heading();
//                 error = targetTicks - currentTicks;

//                 pros::screen::print(pros::E_TEXT_MEDIUM, 0, "Error (Ticks): %.2f", error);

//                 float prop = driveKp * error;
//                 prop = Clamp(prop, -110.0f, 110.0f); // Limited slightly to prevent tire burnout

//                 float deriv = driveKd * (drivePastTicks - currentTicks);

//                 driveIntegral *= 0.95f; 
//                 driveIntegral += error;
//                 driveIntegral = Clamp(driveIntegral, -1000.0f, 1000.0f);
//                 float integ = driveKi * driveIntegral;

//                 PID = prop + deriv + integ;
//                 PID = Clamp(PID, -110.0f, 110.0f);

//                 drivePastTicks = currentTicks;

//                 // Driving values match your forward/backward polarity directions
//                 devices.MGroups.DT_L.set(PID);
//                 devices.MGroups.DT_R.set(PID);

//                 pros::delay(dT);
//         }
//         devices.MGroups.DT_L.set(0);
//         devices.MGroups.DT_R.set(0);
// }
