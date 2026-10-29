#include <algorithm>
#include <array>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

namespace {

constexpr std::array<const char *, 9> motorNames = {
    "MOTOR_L1", "MOTOR_L2", "MOTOR_R1", "MOTOR_R2", "MOTOR_INTAKE_11W",
    "MOTOR_CASCADE_11WR", "MOTOR_CASCADE_11WL", "MOTOR_SCORE_OPEN", "MOTOR_SCORE_ROLL"
};

struct Sample {
    long long timestamp = 0;
    std::array<int, 9> motors{};
};

std::vector<std::string> split(const std::string &line) {
    std::vector<std::string> fields;
    std::stringstream stream(line);
    std::string field;
    while (std::getline(stream, field, ',')) fields.push_back(field);
    return fields;
}

bool parseSample(const std::string &line, Sample &sample) {
    const std::vector<std::string> fields = split(line);
    if (fields.size() < 10) return false;
    try {
        sample.timestamp = std::stoll(fields[0]);
        for (size_t i = 0; i < sample.motors.size(); ++i) {
            sample.motors[i] = std::clamp(std::stoi(fields[i + 1]), -127, 127);
        }
    } catch (const std::exception &) {
        return false;
    }
    return true;
}

void writeMotorCommands(std::ostream &output, const Sample &sample, const std::array<int, 9> &last) {
    for (size_t i = 0; i < sample.motors.size(); ++i) {
        if (sample.motors[i] != last[i]) {
            output << "        devices.Motors." << motorNames[i] << ".set(" << sample.motors[i]
                   << ");\n";
        }
    }
}

bool convert(const std::string &inputPath, const std::string &outputPath) {
    std::ifstream input(inputPath);
    if (!input) {
        std::cerr << "Could not open input file: " << inputPath << '\n';
        return false;
    }
    std::ofstream output(outputPath);
    if (!output) {
        std::cerr << "Could not open output file: " << outputPath << '\n';
        return false;
    }

    std::string line;
    std::getline(input, line); // CSV header
    std::vector<Sample> samples;
    while (std::getline(input, line)) {
        Sample sample;
        if (parseSample(line, sample)) samples.push_back(sample);
    }
    if (samples.empty()) {
        std::cerr << "The input contains no valid recorder samples.\n";
        return false;
    }

    output << "#include \"main.h\"\n"
           << "#include \"hardware.hpp\"\n\n"
           << "namespace auton {\n\n"
           << "void recorded_routine() {\n"
           << "    isAuton = true;\n";

    std::array<int, 9> last{};
    long long previousTimestamp = samples.front().timestamp;
    for (const Sample &sample : samples) {
        const long long delay = std::max(0LL, sample.timestamp - previousTimestamp);
        if (delay > 0) output << "        pros::delay(" << delay << ");\n";
        writeMotorCommands(output, sample, last);
        last = sample.motors;
        previousTimestamp = sample.timestamp;
    }

    output << "        pros::delay(20);\n";
    for (const char *motor : motorNames) {
        output << "        devices.Motors." << motor << ".set(0);\n";
    }
    output << "    isAuton = false;\n"
           << "}\n\n"
           << "} // namespace auton\n";

    std::cout << "Converted " << samples.size() << " samples to " << outputPath << '\n';
    return true;
}

} // namespace

int main(int argc, char **argv) {
    if (argc != 3) {
        std::cerr << "Usage: cppconvertapp <recording.csv> <recorded_auton.cpp>\n";
        return 2;
    }
    return convert(argv[1], argv[2]) ? 0 : 1;
}
