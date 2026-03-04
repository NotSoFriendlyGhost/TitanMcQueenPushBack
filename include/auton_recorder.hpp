#pragma once
#include "api.h"

// Call in initialize() to pre-load a saved recording from the SD card
void auton_recorder_init();

// Call every loop iteration inside opcontrol() — handles record toggle + sampling
void auton_recorder_update();

// Call in autonomous() to replay the last recorded path
void replay_recorded_auton();
