#pragma once

#include "radio_globals.h"

const char* protocolTypeToString(ProtocolType pt);
void printActiveProfileDetails();
void applyProfile(uint8_t profileId);
void speakCurrentProfile();
