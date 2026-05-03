#pragma once
#include "pros/apix.h"

// Call this once in initialize() to register the task
void vision_tracker_init();

// Enable/disable from autonomous
void vision_tracker_enable(bool enable);