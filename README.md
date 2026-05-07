# VEX V5 Simplified Robot Code

5 organized files, one for each control system, with a clear separation between configuration, motion, autonomous, and the entry point.

---

## File Structure

```
simplified-vex-template/
├── config.h           ← Robot settings
├── pid.cpp            ← PID controller math
├── motion.cpp         ← Drive, turn, and odometry functions
├── autonomous.cpp     ← Autonomous routines
├── main.cpp           ← Competition setup
├── makefile           ← Build commands
└── README.md          ← This file
```

---

## Quick Start (5 minutes)

### 1. Hardware Setup
Wire your robot:
- Left Front Motor → Port 1
- Left Back Motor → Port 2
- Right Front Motor → Port 3
- Right Back Motor → Port 4
- Inertial Sensor → Port 5

### 2. Update Robot Settings
Edit config.h:
```cpp
const double WHEEL_DIAMETER_INCHES = 3.25;   // Measure your wheel
const double TRACK_WIDTH_INCHES = 12.0;       // Distance between wheels
```

### 3. Build and Run
```bash
make clean
make
```
Upload to your Brain using the VEX SDK.

### 4. Test
- Press Pre-Auton to calibrate
- Press Driver Control to test with joystick
- Press Autonomous to run the routine

---

## What's in Each File?

| File | Purpose | Edit? |
|------|---------|-------|
| config.h | Motor ports, wheel sizes, PID tuning | Yes |
| pid.cpp | PID math (proportional, integral, derivative) | No |
| motion.cpp | Drive, turn, and odometry functions | No |
| autonomous.cpp | Three autonomous routines | Yes |
| main.cpp | Competition callbacks and setup | No |

---

## Motion Functions

```cpp
// Drive forward or backward
driveDistance(true, 24, 60);   // Forward 24 inches at 60% power
driveDistance(false, 12, 50);  // Backward 12 inches at 50% power

// Turn to a heading (0-360 degrees)
// 0 = forward, 90 = left, 180 = backward, 270 = right
turnToHeading(90, 70);         // Turn left (face 90 degrees) at 70% power

// Stop motors
stopDrivetrain();
```

---

## Custom Functions

These are the reusable custom functions provided by the project:

- driveDistance(bool forward, double inches, double maxPower) drives forward or backward a measured distance using PID feedback.
- turnToHeading(double targetHeading, double maxPower) turns the robot to an absolute heading.
- stopDrivetrain() stops all drivetrain motors.
- updateOdometry() updates the tracked robot position using encoder and inertial readings.
- resetOdometry(double x, double y, double heading) resets the tracked robot position and heading.
- autonomousLeft() runs the left autonomous routine.
- autonomousRight() runs the right autonomous routine.
- autonomousSkills() runs the skills autonomous routine.

---

## Write Your Autonomous

Edit autonomous.cpp. Example:

```cpp
void autonomousRight() {
  // Right side strategy
  driveDistance(true, 24, 60);    // Drive forward 24 inches
  turnToHeading(270, 70);         // Turn right (face 270 degrees)
  driveDistance(true, 18, 60);     // Drive forward 18 inches
  stopDrivetrain();
}
```

Then in main.cpp, pick which routine to run:

```cpp
void autonomous(void) {
  autonomousRight();    // Pick one
  // autonomousLeft();
  // autonomousSkills();
}
```

---

## Tuning PID

Edit config.h for PID constants:

```cpp
const double DRIVE_kP = 2.0;    // Proportional (main controller)
const double DRIVE_kI = 0.1;     // Integral (reach target)
const double DRIVE_kD = 0.05;    // Derivative (smooth and stop shaking)
```

If the robot shakes, lower P or raise D.
If the robot is slow, raise P.
If the robot stops short, raise I.
If the robot overshoots, lower P or raise D.

---

## Troubleshooting

| Problem | Fix |
|---------|-----|
| Motors spin the wrong way | In config.h, change false/true in the motor definitions |
| Robot curves when driving straight | Adjust WHEEL_DIAMETER_INCHES in config.h |
| Robot overshoots target | Lower DRIVE_kP in config.h |
| Robot stops short | Raise DRIVE_kI in config.h |
| Nothing appears on the Brain screen | Make sure the Brain is connected via USB |

---

## That's It

5 organized files. Clear separation of concerns. Edit only what you need to edit.

Good luck.
