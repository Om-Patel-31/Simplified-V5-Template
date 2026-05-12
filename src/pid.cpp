// ============================================================================
// PID CONTROLLER - Feedback control for accurate motion
// ============================================================================
// Students: Don't modify this file. This is the math that makes motion smooth.

#include "vex.h"
#include <cmath>

#include "config.h"
#include "pid.h"

// ============================================================================
// PID CONTROLLER IMPLEMENTATION
// ============================================================================

PID::PID(double p, double i, double d, double tol) 
  : kP(p), kI(i), kD(d), tolerance(tol) {}

// Main PID calculation - returns motor power (-100 to 100)
double PID::calculate(double error, double deltaTime) {
  // Proportional term: respond to current error
  double P = kP * error;
  
  // Integral term: accumulate error over time
  integralSum += error * deltaTime;
  if (integralSum > 50.0) integralSum = 50.0;      // Anti-windup
  if (integralSum < -50.0) integralSum = -50.0;
  double I = kI * integralSum;
  
  // Derivative term: smooth out response
  double D = 0.0;
  if (!firstCall) {
    D = kD * (error - lastError) / deltaTime;
  }
  firstCall = false;
  lastError = error;
  
  // Combine all terms
  double motorPower = P + I + D;
  
  // Clamp to valid motor range
  if (motorPower > MAX_MOTOR_POWER) motorPower = MAX_MOTOR_POWER;
  if (motorPower < -MAX_MOTOR_POWER) motorPower = -MAX_MOTOR_POWER;
  
  return motorPower;
}

void PID::reset() {
  integralSum = 0.0;
  lastError = 0.0;
  firstCall = true;
}

bool PID::isFinished(double error) {
  return std::abs(error) < tolerance;
}
