// Bank 8 keypad actions: baud rate, CI-V address, and the FT-8x7 menu item and row.
#include "ui_keypad_bank.h"
#include "radio_profile.h"

void queryBank8Ft8x7Row() { queryKeypadFt8x7Setting(Ft8x7Setting::Row); }
void queryBank8Ft8x7Menu() { queryKeypadFt8x7Setting(Ft8x7Setting::Menu); }

static bool currentProfileAllowsCivSetup() {
  return currentProtocolType() == PROTO_CIV;
}

void queryBank8CivAddress() {
  printKeypadAction("CIVADDR?");
  if (!currentProfileAllowsCivSetup()) {
    printKeypadStatus("CIVADDR -> unavailable");
    speakKeypadFailure("CIVADDR", KeypadFailure::NotAvailable);
    return;
  }
  char hex[3] = "";
  formatHexByte(currentConnectionProfile().civAddr, hex, sizeof(hex));
  printKeypadStatus("CI {}", hex);
  speakCivAddressValue(currentConnectionProfile().civAddr, false);
}

void beginBank8CivAddressEntry() {
  printKeypadAction("CIVADDR");
  if (!currentProfileAllowsCivSetup()) {
    printKeypadStatus("CIVADDR -> unavailable");
    speakKeypadFailure("CIVADDR", KeypadFailure::NotAvailable);
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

void queryBank8Baud() {
  printKeypadAction("BAUD?");
  const uint32_t baud = currentConnectionProfile().baud;
  printKeypadStatus("BAUD {}", baud);
  speakBaudValue(baud, false);
}

// Where the link's baud sits in bauds; 0 if it is not there.
static int currentBaudIndex(const BaudRates& bauds) {
  const uint32_t baud = currentConnectionProfile().baud;
  for (uint8_t i = 0; i < bauds.count; ++i) {
    if (bauds.rates[i] == baud) return i;
  }
  return 0;
}

// The next (delta > 0) or previous rate the radio offers, wrapping around.
void cycleBank8Baud(int delta) {
  printKeypadAction("BAUD");
  const BaudRates& bauds = currentProfile().link.bauds;
  if (bauds.count < 2) {
    printKeypadStatus("BAUD -> unavailable");
    speakKeypadFailure("BAUD", KeypadFailure::NotAvailable);
    return;
  }
  const int count = bauds.count;
  const int next = (currentBaudIndex(bauds) + (delta < 0 ? count - 1 : 1)) % count;
  const uint32_t baud = bauds.rates[next];
  if (!setCurrentBaud(baud)) return;
  printKeypadStatus("BAUD {}", baud);
  speakBaudValue(baud, true);
}
