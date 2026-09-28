#pragma once

#include "radio_globals.h"

void handleIncomingFrame(const CivFrame& d);
void pumpIncoming(uint32_t maxMs);
