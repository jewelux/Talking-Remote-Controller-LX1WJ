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

// CI-V baud rates the connection setup offers, slowest first.
extern const uint32_t kCivBaudRates[];
extern const size_t kCivBaudRateCount;
// Sets the CI-V address and baud of the current profile, saves them as its
// connection override and reapplies the profile. False when the current profile
// is not a CI-V one.
bool setCurrentCivConnection(uint8_t civAddr, uint32_t baud);
