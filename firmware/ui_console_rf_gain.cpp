#include "ui_console_rf_gain.h"

#include "radio_catalog.h"
#include "radio_features.h"
#include "ui_console.h"
#include "ui_console_internal.h"
#include "ui_features.h"

namespace {

void reply(uint8_t percent) {
  Serial.println(rfGainText(percent));
  speakRfGain(percent);
}

}  // namespace

bool handleConsoleRfGain(const String& line, const String& upper) {
  if (upper != "RFGAIN?" && !upper.startsWith("RFGAIN ")) return false;
  uint8_t percent = 0;
  if (upper == "RFGAIN?") {
    if (reportFeatureFailure(rfGainQuery(percent), "RFGAIN?")) return true;
    reply(percent);
    return true;
  }
  if (upper.startsWith("RFGAIN STEP")) {
    int32_t delta = 0;
    if (!parseStepArg(line, upper, "RFGAIN STEP ", delta)) {
      Serial.println("RFGAIN STEP -> use a signed percent, e.g. RFGAIN STEP -5");
      return true;
    }
    if (reportFeatureFailure(rfGainStep((int)delta, percent), "RFGAIN STEP")) return true;
    reply(percent);
    return true;
  }
  long want = 0;
  if (!parseRangedIntArg(upper, "RFGAIN ", 0, 100, want)) {
    Serial.println("RFGAIN -> use 0..100 (percent), e.g. RFGAIN 50, or STEP <+-n>");
    return true;
  }
  if (reportFeatureFailure(rfGainSet((uint8_t)want, percent), "RFGAIN")) return true;
  reply(percent);
  return true;
}

void printRfGainHelp() {
  if (!rfGainSupported()) return;
  helpLine(currentProfile().caps.setRfGain
               ? "RFGAIN? | RFGAIN <0..100> | RFGAIN STEP <+-n>  (RF gain, percent of the radio's range)"
               : "RFGAIN?  (RF gain, percent of the radio's range)");
}
