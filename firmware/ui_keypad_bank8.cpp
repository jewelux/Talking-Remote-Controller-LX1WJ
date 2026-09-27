// Bank 8 keypad actions: CI-V address and baud rate.
#include "ui_keypad_bank.h"
#include "radio_profile.h"

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
  printKeypadCommand("BANK8 1 SHORT -> CIVADDR?");
  if (!currentProfileAllowsCivSetup()) {
    printKeypadStatus("CIVADDR -> unavailable");
    if (g_speechEnabled) speakNotAvailable();
    return;
  }
  char hex[3] = "";
  formatHexByte(currentProfile().civAddr, hex, sizeof(hex));
  printKeypadStatus(String("CI ") + hex);
  speakCivAddressValue(currentProfile().civAddr, false);
}

void beginBank8CivAddressEntry() {
  printKeypadCommand("BANK8 1 LONG -> CIVADDR");
  if (!currentProfileAllowsCivSetup()) {
    printKeypadStatus("CIVADDR -> unavailable");
    if (g_speechEnabled) speakNotAvailable();
    return;
  }
  keypadInput().beginEntry(InputMode::CivAddrEntry);
  printKeypadStatus("CI ADDRESS PLEASE");
  if (g_speechEnabled) {
    speakToken("c");
    playSilenceMs(50);
    speakToken("i");
    playSilenceMs(80);
    speakToken("please");
  }
}

static int currentBaudIndex() {
  const uint32_t baud = currentProfile().baud;
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
  printKeypadCommand(String("BANK8 2 ") + (delta > 0 ? "SHORT" : "LONG") + " -> BAUD");
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
  if (!setCurrentCivConnection(currentProfile().civAddr, baud)) return;
  printKeypadStatus(String("BAUD ") + String((unsigned long)baud));
  speakBaudValue(baud, true);
}
