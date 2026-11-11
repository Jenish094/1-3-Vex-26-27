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

The recorder starts the odometry task and logs the robot pose at a fixed interval, writing
`timestamp_ms,x_in,y_in,heading_rad`. The converter reads those odometry points, keeps points
that represent at least 1 inch of translation or 0.12 radians of heading change, and emits a
coordinate-following autonomous routine using `robot.follow(...)`. Playback uses the robot's
odometry feedback to correct its path rather than replaying controller inputs, with a timeout
based on the original recording duration.

## Web version

The web version is on https://cppconvert.jenish.tech
