// The keypad's settings engine: entries of a Setting through its SettingOps
// (keypad_setting_ops.cpp), and what a value entry says while it runs.
#include "keypad_labels.h"
#include "keypad_setting_ops.h"
#include "ui_keypad_common.h"
#include "ui_speech.h"

namespace {

// What ops->limits fills in: named after its setting.
ValueSpec specOf(Setting setting) {
  ValueSpec spec;
  spec.name = settingInfo(setting).label;
  spec.setting = setting;
  return spec;
}

}  // namespace

// The radio's value is read once here, so B and C step from it with no more
// radio traffic; Enter writes once.
void settingEntry(Setting setting) {
  const SettingOps* ops = settingOps(setting);
  const char* label = settingInfo(setting).label;
  if (keypadReportIfUnsupported(ops && ops->available(), label)) return;
  printKeypadAction("{} ENTRY", label);
  prepareKeypadSpeechResponse();
  ValueSpec spec = specOf(setting);
  if (keypadReportFeatureFailure(ops->limits(spec), label)) return;
  EntrySeed seed;
  int32_t value = 0;
  const FeatureStatus read = ops->read(value);
  // A radio that cannot say its value still takes one typed.
  if (read != FeatureStatus::Unsupported && keypadReportFeatureFailure(read, label)) return;
  seed.known = read == FeatureStatus::Ok;
  seed.value = value;
  keypadBeginValueEntry(spec, seed);
  if (seed.known) printKeypadStatus("{} PLEASE ({})", label, ops->text(value).c_str());
  else printKeypadStatus("{} PLEASE", label);
  if (const char* words = spokenLabelWords(label)) speakPrompt(words);
}

void keypadValueDigit(const ValueSpec& spec, char key, const char* digits) {
  printKeypadCommand("{} DIGIT -> {}", spec.name, key);
  printKeypadStatus("{} STAGE: {}{}", spec.name, digits, spec.unit);
  if (!g_speechEnabled) return;
  if (key != '*') speakDigitsAndPoint(String(key));
  else speakToken(digits[0] == '-' && !digits[1] ? "minus" : "point");
}

static const SettingOps* valueOps(const ValueSpec& spec) {
  return settingOps(spec.setting);
}

void keypadValueSay(const ValueSpec& spec, int32_t value) {
  const SettingOps* ops = valueOps(spec);
  if (!ops) return;
  printKeypadCommand("{} A", spec.name);
  printKeypadStatus("{} STAGE: {}", spec.name, ops->text(value).c_str());
  ops->speak(value);
}

// Just the value: the entry already said what it is.
void keypadValueStepped(const ValueSpec& spec, int32_t value) {
  const SettingOps* ops = valueOps(spec);
  if (!ops) return;
  printKeypadCommand("{} STEP", spec.name);
  printKeypadStatus("{} STAGE: {}", spec.name, ops->text(value).c_str());
  LabelModeScope bare(LabelMode::Never);
  ops->speak(value);
}

void keypadValueStaged(const ValueSpec& spec, int32_t value) {
  const SettingOps* ops = valueOps(spec);
  if (!ops) return;
  printKeypadStatus("{} STAGE: {} PLEASE", spec.name, ops->text(value).c_str());
  {
    LabelModeScope bare(LabelMode::Never);
    ops->speak(value);
  }
  speakPlease();
}

void keypadValueLimit(const char* name, bool atMax) {
  printKeypadStatus("{} -> {}", name, atMax ? "maximum" : "minimum");
  speakLimit(atMax);
}

void keypadValueCommit(const ValueSpec& spec, int32_t value) {
  const SettingOps* ops = valueOps(spec);
  if (!ops) return;
  printKeypadCommand("ENTER -> {}", ops->text(value).c_str());
  prepareKeypadRadioWrite();
  int32_t readBack = value;
  if (keypadReportFeatureFailure(ops->write(value, readBack), spec.name)) return;
  printKeypadStatus("{}", ops->text(readBack).c_str());
  ops->speak(readBack);
}

// ---- A, B and C on the bank's focus ----

namespace {

// The value the last live step set: the next step of the same setting, with no
// other key between, starts from it and does not read the radio again.
struct RecentStep {
  Setting setting = Setting::None;
  int32_t value = 0;
  uint32_t atMs = 0;
};
RecentStep s_recent;
constexpr uint32_t kRecentStepMs = 3000;

bool recentValue(Setting setting, int32_t& value) {
  if (keypadActiveFocusIsNew() || s_recent.setting != setting) return false;
  if ((uint32_t)(millis() - s_recent.atMs) > kRecentStepMs) return false;
  value = s_recent.value;
  return true;
}

const char* stepWord(int8_t dir, bool big) {
  if (dir > 0) return big ? "UP FAST" : "UP";
  return big ? "DOWN FAST" : "DOWN";
}

EntrySeed knownSeed(int32_t value) {
  EntrySeed seed;
  seed.known = true;
  seed.value = value;
  return seed;
}

// A, B or C with no focus, or a gesture the focus has nothing for.
void reportNotAvailable(const char* label) {
  printKeypadAction("{}", label);
  keypadReportIfUnsupported(false, label);
}

// Sets value and says what the radio read back: named on the first step after
// another key, then just the value.
void writeAndSay(const SettingOps& ops, int32_t value, LabelMode labels) {
  const char* label = settingInfo(ops.setting).label;
  prepareKeypadRadioWrite();
  int32_t readBack = value;
  if (keypadReportFeatureFailure(ops.write(value, readBack), label)) {
    s_recent.setting = Setting::None;
    return;
  }
  s_recent.setting = ops.setting;
  s_recent.value = readBack;
  s_recent.atMs = millis();
  printKeypadStatus("{}", ops.text(readBack).c_str());
  LabelModeScope scope(labels);
  ops.speak(readBack);
}

// The range, then the value now: false, with the failure said, when either is missing.
bool readForStep(const SettingOps& ops, ValueSpec& spec, int32_t& value) {
  const char* label = settingInfo(ops.setting).label;
  if (keypadReportIfUnsupported(ops.available(), label)) return false;
  prepareKeypadSpeechResponse();
  spec = specOf(ops.setting);
  if (keypadReportFeatureFailure(ops.limits(spec), label)) return false;
  if (recentValue(ops.setting, value)) return true;
  return !keypadReportFeatureFailure(ops.read(value), label);
}

// What A, B and C do for a focus that is more than its SettingOps, or has none.
struct FocusActions {
  Setting setting;
  KeyAction say;     // nullptr: read and said through its SettingOps
  KeyAction entry;   // nullptr: settingEntry
  KeyAction centre;  // nullptr: its SettingOps centre
  void (*step)(int8_t dir, bool big);  // nullptr: keypadSettingNudge
};

// Neither the mode nor a profile has a big step.
void stepMode(int8_t dir, bool big) {
  if (big) {
    reportNotAvailable("MODE");
    return;
  }
  nudgeBank1Mode(dir);
}

void stepProfile(int8_t dir, bool big) {
  if (big) {
    reportNotAvailable("PROFILE");
    return;
  }
  if (dir > 0) selectNextProfile();
  else selectPrevProfile();
}

constexpr FocusActions kFocusActions[] = {
    {Setting::RfPower, queryBank1RfPower, beginBank1RfPowerSet, nullptr, nudgeBank1RfPower},
    {Setting::Mode, queryBank1Mode, beginBank1ModeSelect, nullptr, stepMode},
    {Setting::Profile, queryBank9Profile, beginBank9ProfileSelect, resetBank9Profile, stepProfile},
    // A short list: A long steps to the next, as its key does.
    {Setting::SpeechSpeed, queryBank9Speed, stepBank9Speed, nullptr, nullptr},
    {Setting::Agc, queryBank2Agc, beginBank2AgcSelect, nullptr, nudgeBank2Agc},
};

const FocusActions* focusActions(Setting setting) {
  for (const FocusActions& f : kFocusActions) {
    if (f.setting == setting) return &f;
  }
  return nullptr;
}

// A focus with no action of its own for a gesture goes through its SettingOps: false, with
// "not available" said, where it has none.
bool focusHasOps(Setting focus) {
  if (settingOps(focus)) return true;
  reportNotAvailable("FOCUS");
  return false;
}

void stepFocus(int8_t dir, bool big) {
  const Setting focus = keypadActiveFocus();
  const FocusActions* f = focusActions(focus);
  if (f && f->step) f->step(dir, big);
  else if (focusHasOps(focus)) keypadSettingNudge(focus, dir, big);
}

}  // namespace

// A live setting is set at once. A Confirm one (RF power) is read once and its entry opens
// one step away, staged: "power 45 watts please"; its B and C then step with no radio traffic.
void keypadSettingNudge(Setting setting, int8_t dir, bool big) {
  const SettingOps* ops = settingOps(setting);
  const char* label = settingInfo(setting).label;
  printKeypadAction("{} {}", label, stepWord(dir, big));
  if (keypadReportIfUnsupported(ops != nullptr, label)) return;
  ValueSpec spec;
  int32_t value = 0;
  if (!readForStep(*ops, spec, value)) return;
  ValueEntry entry;
  entry.begin(spec, knownSeed(value));
  if (entry.step(dir, big ? spec.bigSteps : 1) != ValueEntry::Step::Moved) {
    keypadValueLimit(label, dir > 0);
    return;
  }
  entry.current(value);
  if (!ops->confirm || !ops->confirm()) {
    writeAndSay(*ops, value, keypadActiveFocusIsNew() ? LabelMode::Always : LabelMode::Never);
    return;
  }
  EntrySeed staged = knownSeed(value);
  staged.staged = true;
  keypadBeginValueEntry(spec, staged);
  printKeypadStatus("{} STAGE: {} PLEASE", label, ops->text(value).c_str());
  {
    LabelModeScope named(LabelMode::Always);
    ops->speak(value);
  }
  speakPlease();
}

void settingSay(Setting setting) {
  const SettingOps* ops = settingOps(setting);
  const char* label = settingInfo(setting).label;
  printKeypadAction("{}?", label);
  if (keypadReportIfUnsupported(ops && ops->available(), label)) return;
  prepareKeypadSpeechResponse();
  int32_t value = 0;
  if (keypadReportFeatureFailure(ops->read(value), label)) return;
  printKeypadStatus("{}", ops->text(value).c_str());
  ops->speak(value);
}

// A short list on a hold: one step up, from the last back to the first, set at once.
void settingNext(Setting setting) {
  const SettingOps* ops = settingOps(setting);
  const char* label = settingInfo(setting).label;
  printKeypadAction("{} NEXT", label);
  if (keypadReportIfUnsupported(ops != nullptr, label)) return;
  ValueSpec spec;
  int32_t value = 0;
  if (!readForStep(*ops, spec, value)) return;
  ValueEntry entry;
  entry.begin(spec, knownSeed(value));
  if (entry.step(1) == ValueEntry::Step::Moved) entry.current(value);
  else value = spec.min;
  writeAndSay(*ops, value, LabelMode::Verbose);
}

// A short says the name, verbose off too.
void focusSay() {
  const Setting focus = keypadActiveFocus();
  LabelModeScope named(LabelMode::Always);
  const FocusActions* f = focusActions(focus);
  if (f && f->say) f->say();
  else if (focusHasOps(focus)) settingSay(focus);
}

void focusEntry() {
  const Setting focus = keypadActiveFocus();
  const FocusActions* f = focusActions(focus);
  if (f && f->entry) f->entry();
  else if (focusHasOps(focus)) settingEntry(focus);
}

void focusCentre() {
  const Setting focus = keypadActiveFocus();
  const FocusActions* f = focusActions(focus);
  if (f && f->centre) {
    f->centre();
    return;
  }
  const SettingOps* ops = settingOps(focus);
  const char* label = settingInfo(focus).label;
  if (!ops || !settingInfo(focus).hasCentre) {
    reportNotAvailable(ops ? label : "FOCUS");
    return;
  }
  printKeypadAction("{} CENTRE", label);
  if (keypadReportIfUnsupported(ops->available(), label)) return;
  writeAndSay(*ops, ops->centre, LabelMode::Always);
}

void focusStep(int8_t dir) { stepFocus(dir, false); }

void focusBigStep(int8_t dir) { stepFocus(dir, true); }
