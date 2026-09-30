// Bank 8 keypad actions: CI-V address and baud rate, and the FT-8x7 menu item and row.
#include "ui_keypad_bank.h"
#include "radio_profile.h"

void queryBank8Ft8x7Row() { queryKeypadFt8x7Setting(Ft8x7Setting::Row); }
void queryBank8Ft8x7Menu() { queryKeypadFt8x7Setting(Ft8x7Setting::Menu); }

static void speakBaudValue(uint32_t baud, bool ok) {
  if (!g_speechEnabled) return;
  speakDigitsAndPoint(String((unsigned long)baud));
  if (ok) {
    playSilenceMs(80);
    speakOk();
  }
}

static bool currentProfileAllowsCivSetup() {
  return currentProtocolType() == PROTO_CIV;
}

void queryBank8CivAddress() {
  printKeypadAction("CIVADDR?");
  if (!currentProfileAllowsCivSetup()) {
    printKeypadStatus("CIVADDR -> unavailable");
    if (g_speechEnabled) speakNotAvailable();
    return;
  }
  char hex[3] = "";
  formatHexByte(currentConnectionProfile().civAddr, hex, sizeof(hex));
  printKeypadStatus(String("CI ") + hex);
  speakCivAddressValue(currentConnectionProfile().civAddr, false);
}

void beginBank8CivAddressEntry() {
  printKeypadAction("CIVADDR");
  if (!currentProfileAllowsCivSetup()) {
    printKeypadStatus("CIVADDR -> unavailable");
    if (g_speechEnabled) speakNotAvailable();
    return;
  }
  keypadBeginEntry(InputMode::CivAddrEntry);
  printKeypadStatus("CI ADDRESS PLEASE");
  if (g_speechEnabled) {
    speakToken("c");
    playSilenceMs(50);
    speakPrompt("i");
  }
}

static int currentBaudIndex() {
  const uint32_t baud = currentConnectionProfile().baud;
  int best = 0;
  uint32_t bestDiff = 0xFFFFFFFFUL;
  for (size_t i = 0; i < kCivBaudRateCount; ++i) {
    uint32_t candidate = kCivBaudRates[i];
    uint32_t diff = (baud > candidate) ? (baud - candidate) : (candidate - baud);
    if (diff < bestDiff) {
      best = (int)i;
      bestDiff = diff;
    }
  }
  return best;
}

void cycleBank8Baud(int delta) {
  printKeypadAction("BAUD");
  if (!currentProfileAllowsCivSetup()) {
    printKeypadStatus("BAUD -> unavailable");
    if (g_speechEnabled) speakNotAvailable();
    return;
  }
  const int count = (int)kCivBaudRateCount;
  int next = currentBaudIndex() + delta;
  if (next < 0) next = count - 1;
  if (next >= count) next = 0;
  const uint32_t baud = kCivBaudRates[next];
  if (!setCurrentCivConnection(currentConnectionProfile().civAddr, baud)) return;
  printKeypadStatus(String("BAUD ") + String((unsigned long)baud));
  speakBaudValue(baud, true);
}
