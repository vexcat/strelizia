#pragma once
#include "okapi/api.hpp"
#include "main.h"

int32_t sonic_dist();
int32_t claw_pos();
int32_t lenc_pos();
int32_t renc_pos();
void init_sensors();
okapi::ADIEncoder& getLEnc();
okapi::ADIEncoder& getREnc();
