#include "main.h"

void intake_in() {
  intake.move(-127);
  boxRoller.move(-127);
}

void intake_out() {
  intake.move(127);
  boxRoller.move(127);
}

void topScore() {
  intake.move(-127);
  boxRoller.move(127);
  topRoller.move(-127);
}

void middleScore() {
  intake.move(-127);
  boxRoller.move(127);
  topRoller.move(127);
}

void stopIntake() {
  intake.brake();
  boxRoller.brake();
  topRoller.brake();
}