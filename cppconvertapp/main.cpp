#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

namespace {

constexpr double minTravelDistanceInches = 3.0;

struct Sample {
    long long timestamp = 0;
    double xIn = 0.0;
    double yIn = 0.0;
    double headingRad = 0.0;
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
    if (fields.size() < 4) return false;
    try {
        sample.timestamp = std::stoll(fields[0]);
        sample.xIn = std::stod(fields[1]);
        sample.yIn = std::stod(fields[2]);
        sample.headingRad = std::stod(fields[3]);
    } catch (const std::exception &) {
        return false;
    }
    return std::isfinite(sample.xIn) && std::isfinite(sample.yIn) &&
           std::isfinite(sample.headingRad);
}

double distanceBetween(const Sample &a, const Sample &b) {
    return std::hypot(b.xIn - a.xIn, b.yIn - a.yIn);
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
        std::cerr << "The input contains no valid odometry samples.\n";
        return false;
    }

    std::vector<Sample> filtered;
    filtered.push_back(samples.front());
    for (std::size_t i = 1; i < samples.size(); ++i) {
        const Sample &candidate = samples[i];
        const Sample &last = filtered.back();
        if (distanceBetween(last, candidate) >= minTravelDistanceInches) {
            filtered.push_back(candidate);
        }
    }

    output << "#include \"main.h\"\n"
           << "#include \"odom.hpp\"\n\n"
           << "namespace auton {\n\n"
           << "void recorded_routine() {\n"
           << "    isAuton = true;\n";

    if (filtered.size() >= 2) {
        output << "    const char *recorded_path = R\"(\n";
        output << std::fixed << std::setprecision(2);
        for (const Sample &sample : filtered) {
            output << "        " << sample.xIn << " " << sample.yIn << "\n";
        }
        output << ")\";\n";
        output << "    robot.follow(recorded_path, 15000.0);\n";
    } else {
        output << "    odomSetPose(" << filtered.front().xIn << ", "
               << filtered.front().yIn << ", " << filtered.front().headingRad << ");\n";
    }

    output << "    isAuton = false;\n"
           << "}\n\n"
           << "} // namespace auton\n";

    std::cout << "Converted " << filtered.size() << " filtered odometry points to "
              << outputPath << '\n';
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
