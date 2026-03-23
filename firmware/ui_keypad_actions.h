#pragma once

#include "radio_globals.h"

void keypadClearAll();
void keypadStageCommand(const String& cmd);
void keypadSendNow(const String& cmd);
void keypadEnter();
void keypadHandleReleased(char k);
