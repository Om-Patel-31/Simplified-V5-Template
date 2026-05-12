#ifndef PID_H
#define PID_H

// ============================================================================
// PID CONTROLLER - Feedback control for accurate motion
// ============================================================================

class PID {
private:
  double kP, kI, kD, tolerance;
  double integralSum = 0.0;
  double lastError = 0.0;
  bool firstCall = true;

public:
  PID(double p, double i, double d, double tol);
  double calculate(double error, double deltaTime);
  void reset();
  bool isFinished(double error);
};

#endif
