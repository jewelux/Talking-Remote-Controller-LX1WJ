#pragma once

#include "radio_globals.h"

uint8_t uiGetBank();
void uiSetBank(uint8_t bank);
bool profileModeFromDigit(char digit, uint8_t& modeOut);
void setTuningSpeechEnabled(bool enabled);
void speakTuningSpeechState();
void speakBankNumber();
void initKeypadUi();
void pollKeypadUi();
