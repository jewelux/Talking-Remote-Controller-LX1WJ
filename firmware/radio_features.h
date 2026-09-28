#pragma once

#include "radio_types.h"

// Radio features the keypad and the console share. Each operation checks the
// profile's capability, talks to the radio, keeps `live` up to date and says
// what happened; the UI that called it prints and speaks the result
// (ui_features.h), so a key and its console command behave the same.

enum class FeatureStatus : uint8_t {
  Ok,
  Unsupported,  // the profile has no such feature
  Timeout,      // the radio did not answer (g_radioReplyTimedOut)
  NoReply,      // a read got no usable answer
  Failed,       // a write was not accepted
};

// Noise reduction. level is 1 or 2 on the TS-480, whose NR has two levels, and
// 0 elsewhere (or when off).
struct NrState {
  bool on = false;
  uint8_t level = 0;
};

// On the TS-480 it gives the level too.
FeatureStatus nrQuery(NrState& out);
FeatureStatus nrSet(bool on);
// How many NR levels the radio has beyond on/off: 2 on the TS-480, else 0.
uint8_t nrLevelCount();
// Sets the NR level, 0 being off. Unsupported on a radio without levels and
// for a level above nrLevelCount().
FeatureStatus nrSetLevel(uint8_t level, NrState& out);
// On, off; on the TS-480 off -> 1 -> 2 -> off. Starts from the tracked state
// when it is known, else reads it.
FeatureStatus nrToggle(NrState& out);

// Noise blanker.
FeatureStatus nbQuery(bool& on);
FeatureStatus nbSet(bool on);
FeatureStatus nbToggle(bool& on);

// Notch filter. width is NOTCH_WIDTH_UNKNOWN when off or not known.
struct NotchState {
  bool on = false;
  NotchWidth width = NOTCH_WIDTH_UNKNOWN;
};

FeatureStatus notchQuery(NotchState& out);
FeatureStatus notchSet(bool on);
// Notch on at width (CI-V only).
FeatureStatus notchSetWidth(NotchWidth width);
// On CI-V off -> NAR -> MID -> WIDE -> off; elsewhere on, off.
FeatureStatus notchToggle(NotchState& out);
