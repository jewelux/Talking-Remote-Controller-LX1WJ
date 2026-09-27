// Bank 8 keypad actions: CI-V address and baud rate.
#include "ui_keypad_bank.h"
#include "radio_prefs.h"
#include "radio_profile.h"

static StoredProfile* mutableCurrentStoredProfile() {
  if (!isValidProfileId(g_profileId)) return nullptr;
  StoredProfile& sp = g_slotProfiles[g_profileId - 1];
  return sp.valid ? &sp : nullptr;
}

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

static void saveAndApplyCurrentConnection() {
  StoredProfile* sp = mutableCurrentStoredProfile();
  if (!sp) return;
  saveConnectionOverrideToNvs(g_profileId, sp->civ.civAddr, sp->civ.baud);
  applyProfile(g_profileId);
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

static const uint32_t kBank8BaudRates[] = {4800, 9600, 19200, 38400, 57600, 115200};

static int currentBaudIndex() {
  const uint32_t baud = currentProfile().baud;
  int best = 0;
  uint32_t bestDiff = 0xFFFFFFFFUL;
  for (size_t i = 0; i < sizeof(kBank8BaudRates) / sizeof(kBank8BaudRates[0]); ++i) {
    uint32_t candidate = kBank8BaudRates[i];
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
  StoredProfile* sp = mutableCurrentStoredProfile();
  if (!sp) return;
  const int count = (int)(sizeof(kBank8BaudRates) / sizeof(kBank8BaudRates[0]));
  int next = currentBaudIndex() + delta;
  if (next < 0) next = count - 1;
  if (next >= count) next = 0;
  sp->civ.baud = kBank8BaudRates[next];
  saveAndApplyCurrentConnection();
  printKeypadStatus(String("BAUD ") + String((unsigned long)sp->civ.baud));
  speakBaudValue(sp->civ.baud, true);
}
