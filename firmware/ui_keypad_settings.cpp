// The keypad's settings engine: entries of a Setting through its SettingOps
// (keypad_setting_ops.cpp), and what a value entry says while it runs.
#include "keypad_setting_ops.h"
#include "ui_keypad_common.h"
#include "ui_speech.h"

// The radio's value is read once here, so B and C step from it with no more
// radio traffic; Enter writes once.
void keypadSettingEntry(Setting setting) {
  const SettingOps* ops = settingOps(setting);
  const char* label = settingInfo(setting).label;
  if (keypadReportIfUnsupported(ops && ops->available(), label)) return;
  printKeypadAction("{} ENTRY", label);
  prepareKeypadSpeechResponse();
  ValueSpec spec;
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
  speakPrompt(ops->prompt);
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
