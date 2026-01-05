#pragma once

#include "EZ-Template/api.hpp"
#include "api.h"

extern Drive chassis;

// Your motors, sensors, etc. should go here.  Below are examples

// inline pros::Motor intake(1);
// inline pros::adi::DigitalIn limit_switch('A');

// Intake motors
inline pros::Motor intake(-15);
inline pros::Motor topMotor(16);

// Pneumatics
inline ez::Piston tongue('B');
inline ez::Piston middleBand('C');
inline ez::Piston descore('A');