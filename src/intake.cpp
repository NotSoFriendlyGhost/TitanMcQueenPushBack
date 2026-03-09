#include "main.h"
#include "subsystems.hpp"

void intake_in() {
  intake.move(127);
  topMotor.move(-20);
  middleBand.set(0);
}

void intake_out() {
  intake.move(-127);
  topMotor.move(-127);
  middleBand.set(0);
}

void middleScore() {
  intake.move(127);
  topMotor.move(127);
  middleBand.set(1);
}

void stopIntake() {
  intake.brake();
  topMotor.brake();
  middleBand.set(0);
  
}

void slopeScore(){
  intake.move(127);
  topMotor.move(127);
  middleBand.set(0);
}