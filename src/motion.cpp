// ============================================================================
// MOTION CONTROL - Drive and turn functions with PID feedback
// ============================================================================
// Students: Don't modify this file. This handles the robot movement math.

#include "vex.h"
#include <cmath>

#include "config.h"
#include "pid.cpp"

vex::motor leftFrontMotor(vex::PORT1, vex::gearSetting::ratio18_1, false);
vex::motor leftBackMotor(vex::PORT2, vex::gearSetting::ratio18_1, false);
vex::motor rightFrontMotor(vex::PORT3, vex::gearSetting::ratio18_1, true);
vex::motor rightBackMotor(vex::PORT4, vex::gearSetting::ratio18_1, true);

vex::controller controller(vex::primary);
vex::inertial inertialSensor(vex::PORT5);

// ============================================================================
// ODOMETRY - TRACK ROBOT POSITION
// ============================================================================

double robotX = 0.0;           // Robot X position in inches
double robotY = 0.0;           // Robot Y position in inches
double robotHeading = 0.0;     // Robot heading in degrees (0-360)

double lastLeftRotations = 0.0;
double lastRightRotations = 0.0;

// Update robot position using motor encoders and inertial sensor
void updateOdometry() {
  // Read current motor rotations
  double leftRotations = leftFrontMotor.rotation(vex::rotationUnits::rev);
  double rightRotations = rightFrontMotor.rotation(vex::rotationUnits::rev);
  
  // Calculate how far each wheel traveled since last update
  double leftDelta = leftRotations - lastLeftRotations;
  double rightDelta = rightRotations - lastRightRotations;
  
  double leftInches = leftDelta * 3.14159 * WHEEL_DIAMETER_INCHES;
  double rightInches = rightDelta * 3.14159 * WHEEL_DIAMETER_INCHES;
  
  // Average distance = how far forward the robot moved
  double avgInches = (leftInches + rightInches) / 2.0;
  
  // Get current heading from inertial sensor
  robotHeading = inertialSensor.heading();
  
  // Update position using simple math
  // If heading is 0°, move forward (+Y)
  // If heading is 90°, move left (+X)
  double headingRad = robotHeading * (3.14159 / 180.0);
  robotX += avgInches * std::sin(headingRad);
  robotY += avgInches * std::cos(headingRad);
  
  lastLeftRotations = leftRotations;
  lastRightRotations = rightRotations;
}

// Reset position to known location (call at start of autonomous)
void resetOdometry(double x, double y, double heading) {
  robotX = x;
  robotY = y;
  robotHeading = heading;
  lastLeftRotations = leftFrontMotor.rotation(vex::rotationUnits::rev);
  lastRightRotations = rightFrontMotor.rotation(vex::rotationUnits::rev);
}

// ============================================================================
// MOTION CONTROL FUNCTIONS
// ============================================================================

// Drive forward or backward a distance in inches
void driveDistance(bool forward, double inches, double maxPower) {
  PID pid(DRIVE_kP, DRIVE_kI, DRIVE_kD, DRIVE_TOLERANCE);
  pid.reset();
  
  // Stop motors before starting
  leftFrontMotor.stop(vex::brakeType::brake);
  leftBackMotor.stop(vex::brakeType::brake);
  rightFrontMotor.stop(vex::brakeType::brake);
  rightBackMotor.stop(vex::brakeType::brake);
  
  double startDistance = std::sqrt(robotX * robotX + robotY * robotY);
  double targetDistance = startDistance + (forward ? inches : -inches);
  
  int elapsedTime = 0;
  
  while (elapsedTime < TIMEOUT_MS) {
    updateOdometry();
    
    double currentDistance = std::sqrt(robotX * robotX + robotY * robotY);
    double error = targetDistance - currentDistance;
    
    double motorPower = pid.calculate(error, 0.02);  // 0.02 = 20ms in seconds
    
    // Cap power to user limit
    if (motorPower > maxPower) motorPower = maxPower;
    if (motorPower < -maxPower) motorPower = -maxPower;
    
    // Send same power to all 4 motors
    leftFrontMotor.spin(vex::directionType::fwd, motorPower, vex::velocityUnits::pct);
    leftBackMotor.spin(vex::directionType::fwd, motorPower, vex::velocityUnits::pct);
    rightFrontMotor.spin(vex::directionType::fwd, motorPower, vex::velocityUnits::pct);
    rightBackMotor.spin(vex::directionType::fwd, motorPower, vex::velocityUnits::pct);
    
    // Debug: Show progress on brain screen
    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1, 1);
    Brain.Screen.print("Distance: error=%.1f in", error);
    
    if (pid.isFinished(error)) {
      // Stop and return
      leftFrontMotor.stop(vex::brakeType::brake);
      leftBackMotor.stop(vex::brakeType::brake);
      rightFrontMotor.stop(vex::brakeType::brake);
      rightBackMotor.stop(vex::brakeType::brake);
      return;
    }
    
    vex::this_thread::sleep_for(CONTROL_LOOP_MS);
    elapsedTime += CONTROL_LOOP_MS;
  }
  
  // Timeout - stop motors
  leftFrontMotor.stop(vex::brakeType::brake);
  leftBackMotor.stop(vex::brakeType::brake);
  rightFrontMotor.stop(vex::brakeType::brake);
  rightBackMotor.stop(vex::brakeType::brake);
}

// Turn to face an absolute heading (0-360 degrees)
void turnToHeading(double targetHeading, double maxPower) {
  PID pid(TURN_kP, TURN_kI, TURN_kD, TURN_TOLERANCE);
  pid.reset();
  
  // Normalize heading to 0-360
  while (targetHeading < 0) targetHeading += 360;
  while (targetHeading >= 360) targetHeading -= 360;
  
  int elapsedTime = 0;
  
  while (elapsedTime < TIMEOUT_MS) {
    updateOdometry();
    
    // Calculate shortest rotation path
    double error = targetHeading - robotHeading;
    if (error > 180) error -= 360;
    if (error < -180) error += 360;
    
    double motorPower = pid.calculate(error, 0.02);
    
    if (motorPower > maxPower) motorPower = maxPower;
    if (motorPower < -maxPower) motorPower = -maxPower;
    
    // Tank drive turn: reverse left, forward right = turn right
    leftFrontMotor.spin(vex::directionType::fwd, -motorPower, vex::velocityUnits::pct);
    leftBackMotor.spin(vex::directionType::fwd, -motorPower, vex::velocityUnits::pct);
    rightFrontMotor.spin(vex::directionType::fwd, motorPower, vex::velocityUnits::pct);
    rightBackMotor.spin(vex::directionType::fwd, motorPower, vex::velocityUnits::pct);
    
    // Debug
    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1, 1);
    Brain.Screen.print("Turn: error=%.1f deg", error);
    
    if (pid.isFinished(error)) {
      leftFrontMotor.stop(vex::brakeType::brake);
      leftBackMotor.stop(vex::brakeType::brake);
      rightFrontMotor.stop(vex::brakeType::brake);
      rightBackMotor.stop(vex::brakeType::brake);
      return;
    }
    
    vex::this_thread::sleep_for(CONTROL_LOOP_MS);
    elapsedTime += CONTROL_LOOP_MS;
  }
  
  // Timeout
  leftFrontMotor.stop(vex::brakeType::brake);
  leftBackMotor.stop(vex::brakeType::brake);
  rightFrontMotor.stop(vex::brakeType::brake);
  rightBackMotor.stop(vex::brakeType::brake);
}

// Stop all motors
void stopDrivetrain() {
  leftFrontMotor.stop(vex::brakeType::brake);
  leftBackMotor.stop(vex::brakeType::brake);
  rightFrontMotor.stop(vex::brakeType::brake);
  rightBackMotor.stop(vex::brakeType::brake);
}
