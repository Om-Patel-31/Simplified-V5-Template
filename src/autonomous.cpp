// ============================================================================
// AUTONOMOUS ROUTINES - MODIFY THESE FOR YOUR GAME
// ============================================================================
// Students: Edit this file to change what your robot does in autonomous!
// You can modify these three routines and pick which one to run.

#include "vex.h"
#include "config.h"

// ============================================================================
// AUTONOMOUS ROUTINES - PICK ONE TO RUN
// ============================================================================

void autonomousLeft() {
  // LEFT SIDE STRATEGY - Edit this!
  driveDistance(true, 24, 60);    // Drive forward 24 inches
  turnToHeading(90, 70);          // Turn left to face 90 degrees
  driveDistance(true, 18, 60);    // Drive forward 18 inches
  stopDrivetrain();
}

void autonomousRight() {
  // RIGHT SIDE STRATEGY - Edit this!
  driveDistance(true, 24, 60);    // Drive forward 24 inches
  turnToHeading(270, 70);         // Turn right to face 270 degrees
  driveDistance(true, 18, 60);    // Drive forward 18 inches
  stopDrivetrain();
}

void autonomousSkills() {
  // SKILLS CHALLENGE - Edit this!
  driveDistance(true, 30, 60);    // Drive forward
  turnToHeading(90, 70);          // Turn left
  driveDistance(true, 24, 60);    // Drive forward
  turnToHeading(180, 70);         // Turn around
  driveDistance(true, 36, 60);    // Drive forward
  stopDrivetrain();
}
