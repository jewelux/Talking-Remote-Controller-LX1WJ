#pragma once

#include "radio_globals.h"

void keypadClearAll();
void keypadStageCommand(const String& cmd);
void keypadSendNow(const String& cmd);
void keypadEnter();
bool keypadApplyFrequencyHz(uint64_t hz, uint8_t targetVfo);
void keypadHandleReleased(char k);
void keypadBank2QueryNr();
void keypadBank2QueryNb();
void keypadBank2QueryNotch();
