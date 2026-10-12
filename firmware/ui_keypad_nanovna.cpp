// NanoVNA keypad actions (KeypadLayout::NanoVna, Bank 1): the sweep range, band presets,
// calibration slot, the version test and the measurement, which nanovna_client.* runs in the
// background and reports here.
#include "ui_keypad_bank.h"
#include "nanovna_client.h"
#include "radio_frequency.h"
#include "ui_keypad.h"

namespace {

String mhzText(uint32_t hz) {
  return RadioFrequency::fromHz(hz).toString();
}

void speakMhz(uint32_t hz) {
  speakDigitsAndPoint(mhzText(hz));
}

// "band 8 0 meters" for a preset, "start 3.6 stop 3.7" for a range of its own.
void speakRange(uint32_t startHz, uint32_t stopHz) {
  const int band = nanoVnaBandMatching(startHz, stopHz);
  if (band >= 0) {
    const NanoVnaBand& b = nanoVnaBandAt((size_t)band);
    printKeypadStatus("BAND {} {}", (unsigned)b.wavelength, b.unit == NanoVnaBandUnit::Meters ? "m" : "cm");
    if (!g_speechEnabled) return;
    speakToken("band");
    speakNumber(String((unsigned)b.wavelength));
    speakToken(b.unit == NanoVnaBandUnit::Meters ? "meters" : "centimeters");
    return;
  }
  printKeypadStatus("RANGE {} - {} MHz", mhzText(startHz).c_str(), mhzText(stopHz).c_str());
  if (!g_speechEnabled) return;
  speakToken("start");
  speakMhz(startHz);
  speakToken("stop");
  speakMhz(stopHz);
}

// "1.21", or "99 plus" for an SWR the meter cannot give.
void speakSwr(float swr) {
  char text[12];
  const bool over = nanoVnaFormatSwr(swr, text, sizeof(text));
  speakNumber(String(text));
  if (over) speakToken("plus");
}

void speakMatch(const char* which, float swr, uint32_t hz) {
  speakToken(which);
  speakToken("match");
  speakSwr(swr);
  speakToken("at");
  speakDigitsAndPoint(hzToMHzString3(hz));
}

void speakMeasurement(const NanoVnaResult& result) {
  const NanoVnaSweepSummary& s = result.summary;
  char best[12];
  char worst[12];
  (void)nanoVnaFormatSwr(s.bestSwr, best, sizeof(best));
  (void)nanoVnaFormatSwr(s.worstSwr, worst, sizeof(worst));
  printKeypadStatus("SWEEP: best SWR {} at {} MHz, worst SWR {} at {} MHz ({} points)", best,
                    hzToMHzString3(s.bestHz).c_str(), worst, hzToMHzString3(s.worstHz).c_str(),
                    (unsigned)s.points);
  if (!g_speechEnabled) return;
  speakMatch("best", s.bestSwr, s.bestHz);
  speakMatch("worst", s.worstSwr, s.worstHz);
}

// A job that did not start, or ended badly: "busy", "usb off", or the label's failure.
void reportNanoVnaStatus(const char* label, NanoVnaStatus status, const char* detail = "") {
  printKeypadStatus("{} -> {}{}{}", label, nanoVnaStatusName(status), detail[0] ? ": " : "", detail);
  switch (status) {
    case NanoVnaStatus::Ok:
      return;
    case NanoVnaStatus::Busy:
      if (g_speechEnabled) speakToken("busy");
      return;
    case NanoVnaStatus::NotConnected:
      if (g_speechEnabled) {
        const char* const tokens[] = {"usb", "off"};
        speakTokens(tokens, 2);
      }
      return;
    case NanoVnaStatus::NotAvailable:
      speakKeypadFailure(label, KeypadFailure::NotAvailable);
      return;
    case NanoVnaStatus::Timeout:
      speakKeypadFailure(label, KeypadFailure::Timeout);
      return;
    case NanoVnaStatus::DeviceError:
    case NanoVnaStatus::NoData:
      speakKeypadFailure(label, KeypadFailure::Error);
      return;
  }
}

void onMeasured(const NanoVnaResult& result) {
  if (result.status != NanoVnaStatus::Ok) {
    reportNanoVnaStatus("SWEEP", result.status, result.text);
    return;
  }
  speakMeasurement(result);
}

void onVersion(const NanoVnaResult& result) {
  if (result.status != NanoVnaStatus::Ok) {
    reportNanoVnaStatus("VERSION", result.status, result.text);
    return;
  }
  printKeypadStatus("VERSION: {}", result.text);
  if (!g_speechEnabled) return;
  char number[24];
  if (!nanoVnaVersionNumber(result.text, number, sizeof(number))) {
    speakOk();
    return;
  }
  speakToken("version");
  speakDigitsAndPoint(String(number));
}

void onBattery(const NanoVnaResult& result) {
  if (result.status != NanoVnaStatus::Ok) {
    reportNanoVnaStatus("BATTERY", result.status, result.text);
    return;
  }
  char volts[12];
  nanoVnaFormatVolts(result.millivolts, volts, sizeof(volts));
  printKeypadStatus("BATTERY: {} V", volts);
  if (!g_speechEnabled) return;
  speakLabel("battery");
  speakNumber(String(volts));
  speakToken("volts");
}

// Digits and letters one by one.
void spellCharacters(const char* text) {
  for (const char* p = text; *p; ++p) {
    if (*p >= '0' && *p <= '9') {
      playDigit(*p - '0');
    } else {
      const char letter[2] = {(char)(*p | 0x20), '\0'};
      speakToken(letter);
    }
  }
}

void onSerialNumber(const NanoVnaResult& result) {
  if (result.status != NanoVnaStatus::Ok) {
    reportNanoVnaStatus("SN", result.status, result.text);
    return;
  }
  char serial[33];
  if (!nanoVnaSerialNumber(result.text, serial, sizeof(serial))) {
    printKeypadStatus("SN -> unexpected reply: {}", result.text);
    speakKeypadFailure("SN", KeypadFailure::Error);
    return;
  }
  printKeypadStatus("SN: {}", serial);
  if (!g_speechEnabled) return;
  speakLabel("s n");
  spellCharacters(serial);
}

void speakCalibration(int8_t slot) {
  if (slot == kNanoVnaCalAuto) {
    printKeypadStatus("CAL: auto");
  } else {
    printKeypadStatus("CAL: slot {}", (int)slot);
  }
  if (!g_speechEnabled) return;
  speakLabel("calibration");
  if (slot == kNanoVnaCalAuto) {
    speakToken("auto");
  } else {
    speakNumber(String((int)slot));
  }
}

void onRecalled(const NanoVnaResult& result) {
  if (result.status != NanoVnaStatus::Ok) {
    reportNanoVnaStatus("CAL", result.status, result.text);
    return;
  }
  speakCalibration(nanoVnaSetup().calSlot);
  speakValueOk();
}

// 'A' picks auto: no recall, the calibration the device has loaded.
constexpr uint8_t kCalAutoValue = 0xFF;

constexpr ChoiceOption kCalKeys[] = {
    {'0', 0}, {'1', 1}, {'2', 2}, {'3', 3}, {'4', 4}, {'5', 5}, {'6', 6}, {'A', kCalAutoValue},
};
static_assert(sizeof(kCalKeys) / sizeof(kCalKeys[0]) == kNanoVnaCalSlots + 1, "a key per slot, and A");

void sayPickedCalibration(char key, uint8_t value) {
  printKeypadCommand("CAL SELECT -> {}", key);
  if (!g_speechEnabled) return;
  if (value == kCalAutoValue) {
    speakToken("auto");
  } else {
    speakNumber(String((unsigned)value));
  }
}

// The slot is kept, and recalled before each measurement. With the NanoVNA there and idle it is
// recalled now too, so a slot the device does not have says "error" at once.
void commitCalibration(uint8_t value, TargetVfo) {
  const int8_t slot = value == kCalAutoValue ? kNanoVnaCalAuto : (int8_t)value;
  printKeypadCommand("ENTER -> CAL {}", value == kCalAutoValue ? "AUTO" : String((unsigned)value).c_str());
  nanoVnaSetCalSlot(slot);
  if (slot != kNanoVnaCalAuto && nanoVnaStartRecall((uint8_t)slot, onRecalled) == NanoVnaStatus::Ok) return;
  speakCalibration(slot);
  speakValueOk();
}

constexpr ChoiceSpec kCalChoice = {
    "CAL SELECT", kCalKeys, sizeof(kCalKeys) / sizeof(kCalKeys[0]), nullptr, sayPickedCalibration,
    commitCalibration};

}  // namespace

void measureNanoVna() {
  printKeypadAction("SWEEP");
  prepareKeypadSpeechResponse();
  const NanoVnaSetup& setup = nanoVnaSetup();
  if (!nanoVnaRangeValid(setup.startHz, setup.stopHz)) {
    printKeypadStatus("SWEEP -> start {} MHz is not below stop {} MHz", mhzText(setup.startHz).c_str(),
                      mhzText(setup.stopHz).c_str());
    speakKeypadFailure("SWEEP", KeypadFailure::Error);
    return;
  }
  const NanoVnaStatus status = nanoVnaStartMeasure(onMeasured);
  if (status != NanoVnaStatus::Ok) {
    reportNanoVnaStatus("SWEEP", status);
    return;
  }
  printKeypadStatus("SWEEP {} - {} MHz", mhzText(setup.startHz).c_str(), mhzText(setup.stopHz).c_str());
  if (g_speechEnabled) speakToken("sweep");
}

void repeatNanoVnaResult() {
  printKeypadAction("SWEEP?");
  prepareKeypadSpeechResponse();
  const NanoVnaResult* last = nanoVnaLastMeasure();
  if (!last) {
    printKeypadStatus("SWEEP? -> no measurement yet");
    speakKeypadFailure("SWEEP", KeypadFailure::NotAvailable);
    return;
  }
  speakMeasurement(*last);
}

void queryNanoVnaStart() {
  printKeypadAction("START?");
  prepareKeypadSpeechResponse();
  const uint32_t hz = nanoVnaSetup().startHz;
  printKeypadStatus("START: {} MHz", mhzText(hz).c_str());
  if (!g_speechEnabled) return;
  speakLabel("start");
  speakMhz(hz);
}

void beginNanoVnaStartEntry() {
  printKeypadAction("START");
  keypadBeginEntry(InputMode::SweepStartEntry);
  speakPrompt("start");
}

void queryNanoVnaStop() {
  printKeypadAction("STOP?");
  prepareKeypadSpeechResponse();
  const uint32_t hz = nanoVnaSetup().stopHz;
  printKeypadStatus("STOP: {} MHz", mhzText(hz).c_str());
  if (!g_speechEnabled) return;
  speakLabel("stop");
  speakMhz(hz);
}

void beginNanoVnaStopEntry() {
  printKeypadAction("STOP");
  keypadBeginEntry(InputMode::SweepStopEntry);
  speakPrompt("stop");
}

void queryNanoVnaRange() {
  printKeypadAction("RANGE?");
  prepareKeypadSpeechResponse();
  speakRange(nanoVnaSetup().startHz, nanoVnaSetup().stopHz);
}

void stepNanoVnaBand(int direction) {
  printKeypadAction("BAND {}", direction > 0 ? "NEXT" : "PREV");
  prepareKeypadSpeechResponse();
  const NanoVnaSetup& setup = nanoVnaSetup();
  const NanoVnaBand& band = nanoVnaBandAt(nanoVnaAdjacentBand(setup.startHz, setup.stopHz, direction));
  nanoVnaSetRange(band.startHz, band.stopHz);
  speakRange(band.startHz, band.stopHz);
}

void queryNanoVnaCalibration() {
  printKeypadAction("CAL?");
  prepareKeypadSpeechResponse();
  speakCalibration(nanoVnaSetup().calSlot);
}

void beginNanoVnaCalibrationSelect() {
  printKeypadAction("CAL");
  keypadBeginChoiceSelect(kCalChoice);
  printKeypadStatus("CAL PLEASE");
  speakPrompt("calibration");
}

void testNanoVna() {
  printKeypadAction("VERSION?");
  prepareKeypadSpeechResponse();
  const NanoVnaStatus status = nanoVnaStartVersion(onVersion);
  if (status != NanoVnaStatus::Ok) reportNanoVnaStatus("VERSION", status);
}

void queryNanoVnaBattery() {
  printKeypadAction("BATTERY?");
  prepareKeypadSpeechResponse();
  const NanoVnaStatus status = nanoVnaStartBattery(onBattery);
  if (status != NanoVnaStatus::Ok) reportNanoVnaStatus("BATTERY", status);
}

void queryNanoVnaSerialNumber() {
  printKeypadAction("SN?");
  prepareKeypadSpeechResponse();
  const NanoVnaStatus status = nanoVnaStartSerialNumber(onSerialNumber);
  if (status != NanoVnaStatus::Ok) reportNanoVnaStatus("SN", status);
}

void keypadCommitNanoVnaFrequency(InputMode mode, const char* digits) {
  const bool start = mode == InputMode::SweepStartEntry;
  const char* label = start ? "START" : "STOP";
  printKeypadCommand("ENTER -> {}", label);
  RadioFrequency parsed;
  if (!RadioFrequency::parseEntry(String(digits), parsed) || !nanoVnaFrequencyValid(parsed.hz())) {
    printKeypadStatus("{} -> invalid", label);
    speakKeypadFailure(label, KeypadFailure::Error);
    return;
  }
  const uint32_t hz = (uint32_t)parsed.hz();
  if (start) {
    nanoVnaSetStart(hz);
  } else {
    nanoVnaSetStop(hz);
  }
  printKeypadStatus("{}: {} MHz", label, mhzText(hz).c_str());
  if (!g_speechEnabled) return;
  speakLabel(start ? "start" : "stop");
  speakMhz(hz);
}
