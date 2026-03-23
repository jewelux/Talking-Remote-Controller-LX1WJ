#pragma once

#include "radio_globals.h"

void handleIncomingFrame(const CivDecoded& d);
void pumpIncoming(uint32_t maxMs);
