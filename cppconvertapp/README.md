# cppconvertapp

This Windows command-line utility converts the robot's SD-card recording into a
replayable PROS C++ routine.

## Build on Windows

From this directory, use CMake:

```powershell
cmake -S . -B build
cmake --build build --config Release
```

The executable is `build\Release\cppconvertapp.exe`.

## Convert a recording

Copy `/usd/recording.csv` from the robot SD to the computer, then run:

```powershell
.\build\Release\cppconvertapp.exe recording.csv recorded_auton.cpp
```

Copy the generated function into `src\auto.cpp` and declare it in
`include\auto.h`. Add it to the `rd::Selector` list in `src\main.cpp` when it
is ready to select from the brain.

The recorder samples every 20 ms. The CSV contains each motor's output command,
controller input, button state, and odometry telemetry. Replay is command-based;
it does not require the controller or SD card to be present during autonomous.
