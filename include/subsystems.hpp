#pragma once

#include "EZ-Template/api.hpp"
#include "api.h"

extern Drive chassis;

// Your motors, sensors, etc. should go here.  Below are examples

// inline pros::Motor intake(1);
// inline pros::adi::DigitalIn limit_switch('A');

// Intake motors
inline pros::Motor intake(-8);

inline pros::Motor topMotor(9);

// Tongue mech
inline ez::Piston tongue('H');

// Middle band
inline ez::Piston middleBand('A');