#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "robotconfig.h" 
#include "robodash/api.h" // IWYU pragma: keep
#include <algorithm>
#include <string>
#include <vector>

extern const lv_img_dsc_t logonat;
extern const lv_img_dsc_t logonatblue;
extern const lv_img_dsc_t logonatred;

rd::Image white(&logonat, "WhiteLogo");
rd::Image blue(&logonatblue, "BlueLogo");
rd::Image red(&logonatred, "RedLogo");
rd::Console location("Location");
rd::Console testMotors("TestMotors");


//Autonomous routines
void redleft(){

}
void redright(){

}
void blueleft(){
}
void blueright(){
	
}



void skills_auton(){

}

void location_auton() {
    location.focus();
}

void testmotors_auton() {
    testMotors.focus();
}

rd::Selector selector({
    {"RedLeft", redleft},
    {"RedRight", redright},
    {"BlueLeft", blueleft},
	{"BlueRight", blueright},
	{"Location", location_auton},
	{"TestMotors", testmotors_auton},
	{"Skills", skills_auton}
});

static std::vector<std::int8_t> get_sorted_motor_ports() {
    const auto motors = pros::Motor::get_all_devices();
    std::vector<std::int8_t> ports;
    ports.reserve(motors.size());

    for (const auto &motor : motors) {
        ports.push_back(motor.get_port());
    }

    std::sort(ports.begin(), ports.end());
    return ports;
}

static std::vector<int> motor_directions;

static void sync_motor_directions(const std::vector<std::int8_t> &motorPorts) {
    if (motor_directions.size() != motorPorts.size()) {
        motor_directions.assign(motorPorts.size(), 0);
    }
}


void initialize() {
    chassis.calibrate();
    selector.on_select([](std::optional<rd::Selector::routine_t> routine) {
        if (!routine) {
            return;
        }

        if (routine->name == "Location") {
            location.focus();
        } else if (routine->name == "TestMotors") {
            testMotors.focus();
        }
    });

    selector.focus();

    pros::Task screenTask([&]() {
        uint32_t last_ui = pros::millis();
        uint32_t last_telemetry = pros::millis();
        uint32_t last_ports_scan = 0;
        std::vector<std::int8_t> motorPorts;

        while (true) {
            const uint32_t now = pros::millis();

            // scan motors less frequently (every 500ms)
            if (now - last_ports_scan > 500) {
                motorPorts = get_sorted_motor_ports();
                sync_motor_directions(motorPorts);
                last_ports_scan = now;
            }

            // UI updates (every 200ms) to reduce LVGL churn
            if (now - last_ui > 200) {
                const auto pose = chassis.getPose();
                location.clear();
                location.println("X: " + std::to_string(pose.x));
                location.println("Y: " + std::to_string(pose.y));
                location.println("Theta: " + std::to_string(pose.theta));

                testMotors.clear();
                if (motorPorts.empty()) {
                    testMotors.println("No motors detected.");
                    testMotors.println("Check ports and wiring.");
                } else {
                    static std::size_t selectedMotor = 0;
                    static bool motorsRunning = false;

                    if (selectedMotor >= motorPorts.size()) {
                        selectedMotor = 0;
                    }

                    if (controller.get_digital_new_press(DIGITAL_UP)) {
                        selectedMotor = (selectedMotor + motorPorts.size() - 1) % motorPorts.size();
                    }
                    if (controller.get_digital_new_press(DIGITAL_DOWN)) {
                        selectedMotor = (selectedMotor + 1) % motorPorts.size();
                    }

                    if (controller.get_digital_new_press(DIGITAL_A)) {
                        motor_directions[selectedMotor] = 1;
                    }
                    if (controller.get_digital_new_press(DIGITAL_Y)) {
                        motor_directions[selectedMotor] = -1;
                    }
                    if (controller.get_digital_new_press(DIGITAL_B)) {
                        motor_directions[selectedMotor] = 0;
                        motorsRunning = false;
                        for (const auto port : motorPorts) {
                            pros::Motor m(port);
                            m.brake();
                        }
                    }
                    if (controller.get_digital_new_press(DIGITAL_X)) {
                        motorsRunning = true;
                    }

                    testMotors.println("TestMotors");
                    testMotors.println("Up/Down: choose motor");
                    testMotors.println("A: select forward");
                    testMotors.println("Y: select reverse");
                    testMotors.println("B: clear/stop");
                    testMotors.println("X: start all selected");
                    testMotors.println("Connected motors:");

                    for (std::size_t index = 0; index < motorPorts.size(); ++index) {
                        const auto port = motorPorts[index];
                        const std::string prefix = index == selectedMotor ? "> Port " : "  Port ";
                        std::string state = "stopped";
                        if (motor_directions[index] > 0) {
                            state = "forward";
                        } else if (motor_directions[index] < 0) {
                            state = "reverse";
                        }

                        testMotors.println(prefix + std::to_string(port) + " [" + state + "]");
                    }

                    // apply motor voltages each UI update (coarse control)
                    for (std::size_t index = 0; index < motorPorts.size(); ++index) {
                        pros::Motor m(motorPorts[index]);
                        const int direction = motor_directions[index];

                        if (motorsRunning && direction != 0) {
                            // run at approx 9V (9000 mV)
                            m.move_voltage(direction > 0 ? 9000 : -9000);
                        } else {
                            m.brake();
                        }
                    }
                }

                last_ui = now;
            }

            // telemetry/logging less frequently (every 1000ms)
            if (now - last_telemetry > 1000) {
                lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());
                last_telemetry = now;
            }

            // light sleep to yield CPU
            pros::delay(10);
        }
    });

} //initialize L bozo american spelling

void disabled() {
	// reset motors sensors and everything between autonomous and driver
}

void competition_initialize() {}

void autonomous() {
    auto selectedAuton = selector.get_auton();
    if (!selectedAuton) {
        return;
    }

    if (selectedAuton->name == "Location" || selectedAuton->name == "TestMotors") {
        return;
    }

    selectedAuton->action();
}

void opcontrol() {
	while (true) {
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        chassis.curvature(rightX, leftY);
        pros::delay(20);                               // Run for 20 ms then update
	}
}
