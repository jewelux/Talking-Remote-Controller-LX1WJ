// Bank 8 keypad actions: baud rate, CI-V address, transceive, and the FT-8x7 menu item and row.
#include "ui_keypad_bank.h"
#include "radio_profile.h"

void queryBank8Ft8x7Row() { queryKeypadFt8x7Setting(Ft8x7Setting::Row); }
void queryBank8Ft8x7Menu() { queryKeypadFt8x7Setting(Ft8x7Setting::Menu); }

// Experimental: "not available" at once unless EXPERIMENTAL is on.
static void beginBank8Ft8x7MenuRowEntry(InputMode mode, const char* label, const char* token) {
  printKeypadAction("{} ENTRY", label);
  if (keypadReportIfUnsupported(currentProfile().caps.setRadioMenu, label)) return;
  keypadBeginEntry(mode);
  printKeypadStatus("{} PLEASE", label);
  speakPrompt(token);
}

void beginBank8Ft8x7RowEntry() { beginBank8Ft8x7MenuRowEntry(InputMode::RowEntry, "ROW", "row"); }
void beginBank8Ft8x7MenuEntry() { beginBank8Ft8x7MenuRowEntry(InputMode::MenuEntry, "MENU", "menu"); }

// Experimental, as the entry: read, then the one next to it written. Past the first or the last
// it says the limit; the last depends on the model and the extended menu, so the radio's
// out-of-range reply finds it.
static void stepBank8Ft8x7MenuRow(Ft8x7Setting setting, int8_t dir, bool big) {
  const bool row = setting == Ft8x7Setting::Row;
  const char* label = row ? "ROW" : "MENU";
  printKeypadAction("{} {}", label, keypadStepWord(dir, big));
  if (keypadReportIfUnsupported(currentProfile().caps.setRadioMenu, label)) return;
  prepareKeypadRadioWrite();
  Ft8x7SettingState state;
  if (keypadReportFeatureFailure(ft8x7SettingQuery(setting, state), label)) return;
  const int step = big ? (row ? 5 : 10) : 1;
  const int next = state.number + (dir > 0 ? step : -step);
  if (next < 1 || next > 255) {
    keypadValueLimit(label, dir > 0);
    return;
  }
  const FeatureStatus status = ft8x7MenuRowSet(setting, (uint8_t)next, state);
  if (status == FeatureStatus::Invalid) {
    keypadValueLimit(label, dir > 0);
    return;
  }
  keypadReportFt8x7Setting(status, label, state);
}

void stepBank8Ft8x7Row(int8_t dir, bool big) { stepBank8Ft8x7MenuRow(Ft8x7Setting::Row, dir, big); }
void stepBank8Ft8x7Menu(int8_t dir, bool big) { stepBank8Ft8x7MenuRow(Ft8x7Setting::Menu, dir, big); }

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

// Sets the baud, keeps it for the profile and says it.
static void applyBaud(uint32_t baud) {
  if (!setCurrentBaud(baud)) {
    keypadReportFailure("BAUD");
    return;
  }
  printKeypadStatus("BAUD {}", baud);
  speakBaudValue(baud, true);
}

// The next or previous rate the radio offers, going round, set at once; there is no big step.
void stepBank8Baud(int8_t dir, bool big) {
  printKeypadAction("BAUD {}", keypadStepWord(dir, big));
  const BaudRates& bauds = currentProfile().link.bauds;
  if (keypadReportIfUnsupported(bauds.count >= 2 && !big, "BAUD")) return;
  const int count = bauds.count;
  applyBaud(bauds.rates[(currentBaudIndex(bauds) + (dir < 0 ? count - 1 : 1)) % count]);
}

// The select's keys: 1 the slowest rate the radio offers, 2 the next, and so on.
static bool pickBaud(char key, uint8_t& value) {
  if (key < '1' || key >= '1' + currentProfile().link.bauds.count) return false;
  value = (uint8_t)(key - '1');
  return true;
}

// Just the rate, as a value entry steps: the prompt already said "baud". A names it.
static void sayPickedBaud(char key, uint8_t value) {
  const uint32_t baud = currentProfile().link.bauds.rates[value];
  printKeypadCommand("BAUD SELECT -> {}", key);
  printKeypadStatus("BAUD STAGE: {}", baud);
  if (key == 'A') speakBaudValue(baud, false);
  else if (g_speechEnabled) speakNumber(String((unsigned long)baud));
}

static void commitBaud(uint8_t value, TargetVfo) {
  const uint32_t baud = currentProfile().link.bauds.rates[value];
  printKeypadCommand("ENTER -> BAUD {}", baud);
  applyBaud(baud);
}

static constexpr ChoiceSpec kBaudChoice = {"BAUD SELECT", nullptr, 0, pickBaud, sayPickedBaud, commitBaud, true};

void beginBank8BaudSelect() {
  printKeypadAction("BAUD");
  const BaudRates& bauds = currentProfile().link.bauds;
  if (keypadReportIfUnsupported(bauds.count >= 2, "BAUD")) return;
  EntrySeed seed;
  seed.known = true;
  seed.value = currentBaudIndex(bauds);
  keypadBeginChoiceSelect(kBaudChoice, TargetVfo::Current, seed);
  printKeypadStatus("BAUD PLEASE");
  speakPrompt("baud");
}

void toggleBank8Transceive() {
  printKeypadAction("TRANSCEIVE");
  if (keypadReportIfUnsupported(protocolSupportsTransceive(), "TRANSCEIVE")) return;
  bool on = false;
  if (!queryTransceiveEnabled(on, kCatReplyTimeoutMs)) { keypadReportFailure("TRANSCEIVE"); return; }
  if (!setTransceiveEnabled(!on)) { keypadReportFailure("TRANSCEIVE"); return; }
  printKeypadStatus("TRANSCEIVE {}", !on ? "ON" : "OFF");
  speakTokenState("transceiver", !on);
}
