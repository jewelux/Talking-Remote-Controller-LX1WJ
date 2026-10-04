#pragma once

#include "radio_globals.h"

const char* protocolTypeToString(ProtocolType pt);
void printActiveProfileDetails();
// Makes the profile in slot the active radio (the default profile for a free
// slot), with the baud and CI-V address saved for it, and reopens the link.
void applyProfile(uint8_t slot);
// SLOTS?: every slot and its radio.
void printProfileSlots();
void speakCurrentProfile();

// Sets the baud of the active profile, saves it for its slot and reopens the
// link. False for a rate the radio does not offer (link.bauds).
bool setCurrentBaud(uint32_t baud);
// Sets the CI-V address of the active profile, saves it for its slot and
// reopens the link. False when the active profile is not a CI-V one.
bool setCurrentCivAddress(uint8_t civAddr);
