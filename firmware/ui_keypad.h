#pragma once

#include "radio_globals.h"

uint8_t uiGetBank();
void uiSetBank(uint8_t bank);
// The mode of a mode select digit '1'-'9' (see user-guide.md).
bool modeFromDigit(char digit, uint8_t& modeOut);
void setTuningSpeechEnabled(bool enabled);
void speakTuningSpeechState();
void setVerboseSpeech(bool verbose);
void speakVerboseState();
void speakExperimentalState();
void speakBankNumber();
void initKeypadUi();
void pollKeypadUi();
