# 41103C 1/3 Code for the Vex V5 2026/2027 Override season
get auto working, 
win states, 
win nationals, 
win taiwan, 
win worlds, 

## Libraries
- Pros
- Dash
- All other libraries were self made

## Jerryio paths
Export path from jerryio using lemlib v0.5 txt then paste it into auto routine using raw string like this:

```cpp
const char example_txt[] = R"(
-36.000, -60.000, 100.000
-36.000, -36.000, 80.000
-12.000, -24.000, 60.000
)";

void planned_auton() {
        robot.follow(example_txt);
}
```