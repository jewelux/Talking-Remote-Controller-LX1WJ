#include "ui_console_rx_filter.h"

#include "radio_catalog.h"
#include "radio_features.h"
#include "rx_filter_codec.h"
#include "ui_console.h"
#include "ui_console_internal.h"
#include "ui_features.h"

namespace {

constexpr RxFilterLevel kLevels[] = {
    RxFilterLevel::IfShift, RxFilterLevel::Width, RxFilterLevel::NotchFreq, RxFilterLevel::ContourFreq,
    RxFilterLevel::ApfFreq, RxFilterLevel::LowCut, RxFilterLevel::HighCut, RxFilterLevel::LowCutSlope,
    RxFilterLevel::HighCutSlope,
};

constexpr RxFilterSwitch kSwitches[] = {RxFilterSwitch::Narrow, RxFilterSwitch::Contour, RxFilterSwitch::Apf};

bool isFt8x7() {
  return currentProtocolType() == PROTO_YAESU_FT8X7;
}

// The word that sets 0: the one it is shown with (NORMAL, OFF), or CENTER for the IF shift and
// the APF.
const char* zeroWord(RxFilterLevel level) {
  if (const char* word = rxFilterZeroWord(level)) return word;
  return level == RxFilterLevel::IfShift || level == RxFilterLevel::ApfFreq ? "CENTER" : nullptr;
}

void reply(RxFilterLevel level, int16_t value) {
  Serial.println(rxFilterLevelText(level, value));
  speakRxFilterLevel(level, value);
}

// "<name> -> invalid (use min..max or OFF)", in the current mode.
void printInvalid(RxFilterLevel level, const char* name) {
  RxFilterRange range;
  if (reportFeatureFailure(rxFilterLevelRange(level, range), name)) return;
  Serial.print(name);
  if (rxFilterLevelIsSlope(level)) {
    Serial.println(" -> invalid (use 6 or 18)");
    return;
  }
  Serial.print(" -> invalid (use ");
  Serial.print(range.min);
  Serial.print("..");
  Serial.print(range.max);
  if (range.zero) {
    Serial.print(" or ");
    Serial.print(zeroWord(level));
  }
  Serial.println(")");
}

// <name>? | <name> <value> | <name> <zero word> | <name> STEP <+-n>.
bool handleLevel(RxFilterLevel level, const String& line, const String& upper) {
  const String name = rxFilterLevelName(level);
  const String prefix = name + " ";
  if (upper != name + "?" && !upper.startsWith(prefix)) return false;
  int16_t value = 0;
  if (upper == name + "?") {
    if (reportFeatureFailure(rxFilterLevelQuery(level, value), (name + "?").c_str())) return true;
    reply(level, value);
    return true;
  }
  const String stepPrefix = prefix + "STEP ";
  if (upper.startsWith(stepPrefix)) {
    int32_t delta = 0;
    if (!parseStepArg(line, upper, stepPrefix.c_str(), delta)) {
      Serial.println(name + " STEP -> use a signed number of steps, e.g. " + name + " STEP -1");
      return true;
    }
    if (delta > 100) delta = 100;
    if (delta < -100) delta = -100;
    const FeatureStatus status = rxFilterLevelStep(level, (int)delta, value);
    // The width NORMAL has no place among the widths, so no step from it.
    const bool fromNormal = level == RxFilterLevel::Width && status == FeatureStatus::Invalid;
    const String label = fromNormal ? name + " STEP (set a width first)" : name;
    if (reportFeatureFailure(status, label.c_str())) return true;
    reply(level, value);
    return true;
  }
  long want = 0;
  const char* zero = zeroWord(level);
  if (zero && upper == prefix + zero) {
    want = 0;
  } else if (!parseRangedIntArg(upper, prefix.c_str(), -32768, 32767, want)) {
    Serial.print(name + " -> use " + (rxFilterLevelIsSlope(level) ? "6 or 18 (dB/oct)" : "a number in Hz"));
    if (zero) Serial.print(String(", ") + zero);
    Serial.println(", or STEP <+-n>");
    return true;
  }
  const FeatureStatus status = rxFilterLevelSet(level, (int16_t)want, value);
  if (status == FeatureStatus::Invalid) {
    printInvalid(level, name.c_str());
    return true;
  }
  if (reportFeatureFailure(status, name.c_str())) return true;
  reply(level, value);
  return true;
}

// <name>? | <name> ON | OFF | TOGGLE.
bool handleSwitch(RxFilterSwitch sw, const String& upper) {
  const String name = rxFilterSwitchName(sw);
  bool on = false;
  FeatureStatus status;
  const bool write = upper == name + " ON" || upper == name + " OFF" || upper == name + " TOGGLE";
  if (write && isFt8x7() && rxFilterSwitchSupported(sw)) {
    // The FT-857/897 keeps NAR in its band block, which HamTRC only reads.
    reportNotAvailable((name + " -> read only on this radio").c_str());
    return true;
  }
  if (upper == name + "?") status = rxFilterSwitchQuery(sw, on);
  else if (upper == name + " ON" || upper == name + " OFF") status = rxFilterSwitchSet(sw, upper == name + " ON", on);
  else if (upper == name + " TOGGLE") status = rxFilterSwitchToggle(sw, on);
  else return false;
  if (reportFeatureFailure(status, upper.endsWith("?") ? (name + "?").c_str() : name.c_str())) return true;
  Serial.println(rxFilterSwitchText(sw, on));
  speakRxFilterSwitch(sw, on);
  return true;
}

}  // namespace

bool handleConsoleRxFilter(const String& line, const String& upper) {
  for (RxFilterLevel level : kLevels) {
    if (handleLevel(level, line, upper)) return true;
  }
  for (RxFilterSwitch sw : kSwitches) {
    if (handleSwitch(sw, upper)) return true;
  }
  return false;
}

void printRxFilterHelp() {
  const bool yaesu = rxFilterEncodingIsYaesu(currentProfile().settings.rxFilter);
  if (rxFilterLevelSupported(RxFilterLevel::Width)) {
    helpLine(yaesu ? "WIDTH? | WIDTH <Hz> | NORMAL | STEP <+-n>  (filter width of the mode, to its nearest width; not AM, FM)"
                   : "WIDTH? | WIDTH <Hz> | STEP <+-n>  (width of the current filter, to its nearest width; not FM)");
  }
  if (rxFilterSwitchSupported(RxFilterSwitch::Narrow)) {
    helpLine(isFt8x7() ? "NAR?  (narrow, of the current band)" : "NAR? | NAR OFF | ON | TOGGLE  (narrow)");
  }
  if (rxFilterLevelSupported(RxFilterLevel::IfShift)) {
    helpLine("IFSHIFT? | IFSHIFT <-1200..1200> | CENTER | STEP <+-n>  (Hz, 20 Hz steps)");
  }
  if (rxFilterLevelSupported(RxFilterLevel::NotchFreq)) {
    helpLine("NOTCHFREQ? | NOTCHFREQ <10..3200> | STEP <+-n>  (manual notch, Hz, 10 Hz steps)");
  }
  if (rxFilterSwitchSupported(RxFilterSwitch::Contour)) helpLine("CONTOUR? | CONTOUR OFF | ON | TOGGLE");
  if (rxFilterLevelSupported(RxFilterLevel::ContourFreq)) {
    helpLine("CONTOURFREQ? | CONTOURFREQ <10..3200> | STEP <+-n>  (Hz, 10 Hz steps)");
  }
  if (rxFilterSwitchSupported(RxFilterSwitch::Apf)) helpLine("APF? | APF OFF | ON | TOGGLE  (audio peak filter, CW)");
  if (rxFilterLevelSupported(RxFilterLevel::ApfFreq)) {
    helpLine("APFFREQ? | APFFREQ <-250..250> | CENTER | STEP <+-n>  (Hz from the pitch, 10 Hz steps)");
  }
  if (rxFilterLevelSupported(RxFilterLevel::LowCut)) {
    if (isFt8x7()) {
      helpLine("HPF? | LPF?  (DSP low and high cut, menus 46 and 47)");
      helpLine("HPF <100..1000> | LPF <1000..6000> | HPF STEP | LPF STEP <+-n>  (experimental: Hz to the nearest step)");
    } else if (yaesu) {
      helpLine("HPF? | HPF <100..1000> | OFF | STEP <+-n>  (low cut of the mode, a menu setting, 50 Hz steps)");
      helpLine("LPF? | LPF <700..4000> | OFF | STEP <+-n>  (high cut of the mode, a menu setting, 50 Hz steps)");
    } else {
      helpLine("HPF? | HPF <Hz> | OFF | STEP <+-n>  (slope tune low cut in SSB, FM and AM, to its nearest step)");
      helpLine("LPF? | LPF <Hz> | STEP <+-n>  (slope tune high cut in SSB, FM and AM, to its nearest step)");
    }
  }
  if (rxFilterLevelSupported(RxFilterLevel::LowCutSlope)) {
    helpLine("HPFSLOPE? | HPFSLOPE 6 | 18 | LPFSLOPE? | LPFSLOPE 6 | 18  (cut slope of the mode, dB/oct)");
  }
}
