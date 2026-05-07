#ifndef CONFIG_H
#define CONFIG_H

#include "vex.h"

// Robot hardware shared by the motion modules.
extern vex::brain Brain;
extern vex::motor leftFrontMotor;
extern vex::motor leftBackMotor;
extern vex::motor rightFrontMotor;
extern vex::motor rightBackMotor;
extern vex::controller controller;
extern vex::inertial inertialSensor;

// Odometry state.
extern double robotX;
extern double robotY;
extern double robotHeading;

// Robot tuning values.
constexpr double WHEEL_DIAMETER_INCHES = 3.25;
constexpr double TRACK_WIDTH_INCHES = 12.0;
constexpr double DRIVE_kP = 2.0;
constexpr double DRIVE_kI = 0.1;
constexpr double DRIVE_kD = 0.05;
constexpr double TURN_kP = 2.0;
constexpr double TURN_kI = 0.1;
constexpr double TURN_kD = 0.05;
constexpr double DRIVE_TOLERANCE = 1.0;
constexpr double TURN_TOLERANCE = 2.0;
constexpr int TIMEOUT_MS = 4000;
constexpr int CONTROL_LOOP_MS = 20;
constexpr double MAX_MOTOR_POWER = 100.0;

void driveDistance(bool forward, double inches, double maxPower);
void turnToHeading(double targetHeading, double maxPower);
void stopDrivetrain();
void updateOdometry();
void resetOdometry(double x, double y, double heading);

void autonomousLeft();
void autonomousRight();
void autonomousSkills();

#endif