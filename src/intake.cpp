#include "main.h"
#include "subsystems.hpp"

void intake_in() {
  intake.move(127);
  topMotor.move(-20);
}

void intake_out() {
  intake.move(-127);
  topMotor.move(-127);
}

void topScore() {
  intake.move(-127);
  // boxRoller.move(127);
  // topRoller.move(-127);
}

void middleScore() {
  intake.move(127);
  topMotor.move(127);
  middleBand.set(1);
  // boxRoller.move(127);
  // topRoller.move(127);
}

void stopIntake() {
  intake.brake();
  topMotor.brake();
}

void slopeScore(){
  intake.move(127);
  topMotor.move(127);
}

void colorSortLaunch(){
  intake.move(127);
  topMotor.move(-127);
}