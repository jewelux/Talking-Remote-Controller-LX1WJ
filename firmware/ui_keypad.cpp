// Keypad glue: feeds Keypad library events to the KeypadInput state machine and
// carries out what it decides through the keymap (keypad_keymap.cpp), the bank
// actions (ui_keypad_bankN.cpp) and the entries (ui_keypad_entry.cpp).
#include "ui_keypad.h"
#include "keypad_input.h"
#include "keypad_keymap.h"
#include "protocol_ascii.h"
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
bool g_suppressModePrefixOnce = false;
// Quiet window after a key press so a tuning announcement cannot start while the
// key's own response (deferred by double-click detection) is being prepared.
static constexpr uint32_t KEYPAD_PRESS_SPEECH_QUIET_MS = 1000;

// Command staged by a Bank 1 query when AUTO_SEND_BANK1_QUERIES is off; Enter
// sends it.
static String s_stagedCommand;
static bool s_hasStagedCommand = false;

static void speakBankPlease() {
  if (!g_speechEnabled) return;
  speakToken("bank");
  playSilenceMs(60);
  speakToken("please");
}

bool profileModeFromDigit(char digit, uint8_t& modeOut) {
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
  String code;
  return profileModeCodeForInternal(currentStoredProfile(), modeOut, code);
}

void speakBankNumber() {
  if (!g_speechEnabled) return;
  const uint8_t bank = keypadInput().bank();
  speakToken("bank");
  playSilenceMs(60);
  if (bank >= 1 && bank <= 9) playDigit(bank);
}

// Any key press interrupts the device: the user wants the answer to this key,
// not whatever was still being spoken or waiting to be spoken. This is the only
// place keypad speech is interrupted; key actions compose their answer (label,
// then value) by appending, so nothing they queue is cut off.
static void silenceSpeechForKeyPress() {
  audioAbortNow();
  cancelPendingFreqAnnouncement();
  g_suppressFreqSpeakUntilMs = millis() + KEYPAD_PRESS_SPEECH_QUIET_MS;
}

static KeypadTraits currentKeypadTraits() {
  KeypadTraits t;
  t.civ = currentProtocolType() == PROTO_CIV;
  t.ftdx10 = isFtdx10KeypadProfile();
  t.ft817 = isFt8x7Ft817Keypad();
  t.ft857Family = isFt8x7Ft857FamilyKeypad();
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
  g_suppressFreqSpeakUntilMs = millis() + 2000;
  cancelPendingFreqAnnouncement();
  g_suspendPollingUntilMs = millis() + 900;
  g_keypadExecuting = true;
  processCommand(cmd);
  g_keypadExecuting = false;
  g_suspendPollingUntilMs = millis() + 900;
}

void keypadStageCommand(const String& cmd) {
  s_stagedCommand = cmd;
  s_hasStagedCommand = true;
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

namespace {

class KeypadUiListener : public KeypadInputListener {
 public:
  void onActivity(bool pressed) override {
    // Actions run on release, hold or after the double-click wait, with
    // background polling in between: a timeout it left must not be blamed on
    // this key.
    g_radioReplyTimedOut = false;
    if (pressed) silenceSpeechForKeyPress();
    traits_ = currentKeypadTraits();
  }

  bool runHold(uint8_t bank, char key) override { return keymapHold(traits_, bank, key); }
  bool runShort(uint8_t bank, char key) override { return keymapShort(traits_, bank, key); }
  bool runDoubleClick(uint8_t bank, char key) override { return keymapDoubleClick(traits_, bank, key); }
  bool wantsDoubleClick(uint8_t bank, char key) override { return keymapWantsDoubleClick(traits_, bank, key); }
  bool runModeSelectShort(uint8_t bank, char key) override { return keymapModeSelectShort(traits_, bank, key); }
  bool runModeSelectHold(uint8_t bank, char key) override { return keymapModeSelectHold(traits_, bank, key); }

  void onBankQuery(uint8_t bank) override {
    printKeypadCommand("* SHORT -> BANK?");
    printKeypadStatus(String("BANK ") + String((int)bank));
    speakBankNumber();
  }

  void onBankSelectStart() override {
    printKeypadCommand("* HOLD -> BANK SELECT");
    printKeypadStatus("BANK PLEASE");
    speakBankPlease();
  }

  void onDigitAccepted(InputMode mode, char key, const char* digits) override {
    keypadEntryDigit(mode, key, digits);
  }

  void onUnassigned(const char* label) override { keypadReportUnassigned(label); }

  void onCommit(InputMode mode, const char* digits, uint8_t targetVfo) override {
    keypadEntryCommit(mode, digits, targetVfo);
  }

  bool onModeDigit(char key, uint8_t& mode) override { return keypadModeDigit(key, mode); }
  void onModeCommit(uint8_t mode, uint8_t targetVfo) override { keypadModeCommit(mode, targetVfo); }

  bool sendStagedCommand() override {
    if (!s_hasStagedCommand) return false;
    keypadSendNow(s_stagedCommand);
    s_stagedCommand = "";
    s_hasStagedCommand = false;
    return true;
  }

  void onClear() override {
    s_stagedCommand = "";
    s_hasStagedCommand = false;
    printKeypadCommand("CLEAR");
    if (g_speechEnabled) speakToken("cancel");
  }

 private:
  KeypadTraits traits_;
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

void keypadEvent(KeypadEvent k) {
  s_input.onKey((char)k, gestureFor(keypad.getState()), millis());
}

}  // namespace

KeypadInput& keypadInput() {
  return s_input;
}

void initKeypadUi() {
  keypad.setDebounceTime(KEYPAD_DEBOUNCE_MS);
  keypad.setHoldTime(KEYPAD_HOLD_MS);
  keypad.addEventListener(keypadEvent);
}

void pollKeypadUi() {
  (void)keypad.getKey();
  s_input.poll(millis());
}

uint8_t uiGetBank() {
  return s_input.bank();
}

void uiSetBank(uint8_t bank) {
  s_input.setBank(bank);
}
