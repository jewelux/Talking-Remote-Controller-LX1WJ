// Keypad glue: feeds Keypad library events to the KeypadInput state machine and
// carries out what it decides through the keymap (keypad_keymap.cpp), the bank
// actions (ui_keypad_bankN.cpp) and the entries (ui_keypad_entry.cpp).
#include "ui_keypad.h"
#include "keypad_input.h"
#include "keypad_keymap.h"
#include "radio_catalog.h"
#include "radio_monitor.h"
#include "radio_profile.h"
#include "radio_protocol.h"
#include "radio_state.h"
#include "ui_console.h"
#include "ui_keypad_common.h"
#include "ui_speech.h"

extern Keypad keypad;

bool g_keypadExecuting = false;
// Quiet window after a key press so a tuning announcement cannot start while the
// key's own response (deferred by double-click detection) is being prepared.
static constexpr uint32_t KEYPAD_PRESS_SPEECH_QUIET_MS = 1000;

bool modeFromDigit(char digit, uint8_t& modeOut) {
  switch (digit) {
    case '1': modeOut = 0x00; break;
    case '2': modeOut = 0x01; break;
    case '3': modeOut = 0x03; break;
    case '4': modeOut = 0x02; break;
    case '5': modeOut = 0x05; break;
    case '6': modeOut = 0x11; break;
    case '7': modeOut = 0x04; break;
    case '8': modeOut = 0x07; break;
    case '9': modeOut = 0x08; break;
    default: return false;
  }
  return true;
}

void speakBankNumber() {
  if (!g_speechEnabled) return;
  const uint8_t bank = uiGetBank();
  speakToken("bank");
  playSilenceMs(60);
  if (bank >= 1 && bank <= 9) playDigit(bank);
}

// A radio poll blocks the loop, and keys are only scanned between polls. While
// the user is pressing keys, no poll runs; an existing longer pause is kept.
static void suspendPollingForKeypad() {
  const uint32_t until = millis() + KEYPAD_PRESS_SPEECH_QUIET_MS;
  if ((int32_t)(until - g_suspendPollingUntilMs) > 0) g_suspendPollingUntilMs = until;
}

// Any key press interrupts the device: the user wants the answer to this key,
// not whatever was still being spoken or waiting to be spoken. This is the only
// place keypad speech is interrupted; key actions compose their answer (label,
// then value) by appending, so nothing they queue is cut off.
static void silenceSpeechForKeyPress() {
  audioAbortNow();
  cancelPendingFreqAnnouncement();
  g_suppressFreqSpeakUntilMs = millis() + KEYPAD_PRESS_SPEECH_QUIET_MS;
  suspendPollingForKeypad();
}

static KeypadTraits currentKeypadTraits() {
  KeypadTraits t;
  if (isFtdx10KeypadProfile()) t.layout = KeypadLayout::Ftdx10;
  else if (isFt8x7Ft817Keypad()) t.layout = KeypadLayout::Ft817;
  else if (isFt8x7Ft857FamilyKeypad()) t.layout = KeypadLayout::Ft857;
  else if (currentProtocolType() == PROTO_YAESU_FT8X7) t.layout = KeypadLayout::Ft8x7;
  else if (currentProtocolType() == PROTO_CIV) t.layout = KeypadLayout::Civ;
  t.lightIcomFallback = lightIcomFallbackActive();
  t.supportsMonitor = protocolSupportsMonitor();
  t.supportsTransceive = protocolSupportsTransceive();
  t.canGetRfPower = currentStoredProfile().caps.getRfPower;
  return t;
}

void keypadSendNow(const String& cmd) {
  if ((bool)Serial) {
    Serial.print("CMD SEND ");
    Serial.println(cmd);
  }
  prepareKeypadSpeechResponse();
  g_keypadExecuting = true;
  processCommand(cmd);
  g_keypadExecuting = false;
  holdKeypadPolling();
}

namespace {

class KeypadUiListener : public KeypadInputListener {
 public:
  void onActivity(bool pressed) override {
    // Actions run on release, hold or after the double-click wait (in entries
    // and selections on press), with background polling in between: a timeout
    // it left must not be blamed on this key.
    g_radioReplyTimedOut = false;
    if (pressed) silenceSpeechForKeyPress();
  }

  KeyBinding keyBinding(uint8_t bank, char key) override {
    return keymapLookup(currentKeypadTraits(), bank, key);
  }

  void onBankQuery(uint8_t bank) override {
    printKeypadCommand("* SHORT -> BANK?");
    printKeypadStatus(String("BANK ") + String((int)bank));
    speakBankNumber();
  }

  void onBankSelectStart() override {
    printKeypadCommand("* HOLD -> BANK SELECT");
    printKeypadStatus("BANK PLEASE");
    speakPrompt("bank");
  }

  // Just the digit: "bank N" would sound like a lasting switch.
  void onOneShotBank(uint8_t bank) override {
    printKeypadCommand(String("BANK SELECT DIGIT -> ONCE ") + String((int)bank));
    printKeypadStatus(String("BANK ") + String((int)bank) + " ONCE");
    if (g_speechEnabled) playDigit(bank);
  }

  void onDigitAccepted(const EntrySpec& entry, char key, const char* digits) override {
    keypadEntryDigit(entry, key, digits);
  }

  void onRejected(const char* label) override { keypadReportUnassigned(label); }

  void onCommit(InputMode mode, const char* digits, TargetVfo targetVfo) override {
    keypadEntryCommit(mode, digits, targetVfo);
  }

  bool onModeDigit(char key, uint8_t& mode) override { return keypadModeDigit(key, mode); }
  void onModeCommit(uint8_t mode, TargetVfo targetVfo) override { keypadModeCommit(mode, targetVfo); }

  void onStagedCommandSend(const char* cmd) override { keypadSendNow(cmd); }

  void onClear() override {
    printKeypadCommand("CLEAR");
    if (g_speechEnabled) speakToken("cancel");
  }
};

KeypadUiListener s_listener;
KeypadInput s_input(s_listener);

KeyGesture gestureFor(KeyState state) {
  switch (state) {
    case PRESSED: return KeyGesture::Pressed;
    case HOLD: return KeyGesture::Held;
    case RELEASED: return KeyGesture::Released;
    default: return KeyGesture::Idle;
  }
}

// getKeys() reports every key that changes state, so KeypadInput sees keys
// pressed together. getState() is only the first key's, hence the lookup.
void keypadEvent(KeypadEvent k) {
  const int slot = keypad.findInList((char)k);
  if (slot < 0) return;
  s_input.onKey((char)k, gestureFor(keypad.key[slot].kstate), millis());
}

}  // namespace

void keypadStageCommand(const String& cmd) {
  s_input.stageCommand(cmd.c_str());
  if ((bool)Serial) {
    Serial.print("CMD STAGE ");
    Serial.println(cmd);
  }
  if (g_speechEnabled) {
    speakKeypadCommandWord(cmd);
    playSilenceMs(60);
    speakToken("ok");
  }
}

void keypadBeginEntry(InputMode mode, TargetVfo targetVfo) {
  s_input.beginEntry(mode, targetVfo);
}

void keypadBeginModeSelect(TargetVfo targetVfo) {
  s_input.beginModeSelect(targetVfo);
}

void keypadBeginProfileSelect() {
  s_input.beginProfileSelect();
}

void initKeypadUi() {
  keypad.setDebounceTime(KEYPAD_DEBOUNCE_MS);
  keypad.setHoldTime(KEYPAD_HOLD_MS);
  keypad.addEventListener(keypadEvent);
}

void pollKeypadUi() {
  (void)keypad.getKeys();
  s_input.poll(millis());
  // An open entry or selection waits for more keys: keep the radio unpolled.
  if (s_input.mode() != InputMode::Normal) suspendPollingForKeypad();
}

uint8_t uiGetBank() {
  return s_input.bank();
}

void uiSetBank(uint8_t bank) {
  s_input.setBank(bank);
}
