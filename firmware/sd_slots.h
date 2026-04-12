#pragma once

#include "radio_globals.h"

enum SdLoadStatus : uint8_t {
  SD_LOAD_OK = 0,
  SD_LOAD_INIT_FAILED,
  SD_LOAD_SLOTS_FILE_MISSING,
  SD_LOAD_NO_VALID_PROFILES
};

bool loadProfilesFromSd();
void printProfileSlots();
SdLoadStatus getLastSdLoadStatus();
const char* getLastSdLoadStatusText();
