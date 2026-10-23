#pragma once

#include <Arduino.h>

#include "cat_setting_codec.h"

uint64_t decodeBcdFrequencyHz(const uint8_t* bcd, size_t len);
String hzToMHzString3(uint64_t hz);
int32_t bcdDigitsToInt(const uint8_t* b, size_t n);
uint8_t smRawToS(int32_t raw);
float swrRawToValue(int32_t raw);
