#include "radio_catalog.h"

static const RadioProfile* s_base = profileForSlot(kDefaultProfileSlot);
static RadioProfile s_active = *s_base;
static ConnectionProfile s_connection = {s_base->link.port, s_base->link.baud, s_base->link.civAddr,
                                         SerialFraming::Standard};

// RadioCapabilities has only bool members, so it can be walked as a bool array.
static RadioCapabilities allCapsOn() {
  RadioCapabilities caps{};
  bool* flags = reinterpret_cast<bool*>(&caps);
  for (size_t i = 0; i < sizeof(RadioCapabilities) / sizeof(bool); ++i) flags[i] = true;
  return caps;
}

static void applyCaps() {
  s_active.caps = g_experimentalCaps ? allCapsOn() : s_base->caps;
}

void selectActiveProfile(const RadioProfile& profile, uint32_t baud, uint8_t civAddr) {
  s_base = &profile;
  s_active = profile;
  applyCaps();
  const bool yaesu5ByteCat = profile.protocol == PROTO_YAESU_FT8X7 || profile.protocol == PROTO_YAESU_FT847;
  const SerialFraming framing = yaesu5ByteCat ? SerialFraming::Ft8x7Cat : SerialFraming::Standard;
  s_connection = {profile.link.port, baud, civAddr, framing};
}

const RadioProfile& currentProfile() {
  return s_active;
}

void setExperimentalCaps(bool on) {
  g_experimentalCaps = on;
  applyCaps();
}

const ConnectionProfile& currentConnectionProfile() {
  return s_connection;
}

ProtocolType currentProtocolType() {
  return s_active.protocol;
}

RadioModel currentRadioModel() {
  return s_active.model;
}

Ft8x7Model currentFt8x7Model() {
  return ft8x7ModelFor(s_active.model);
}

bool currentIsFt817Family() {
  return ft8x7IsFt817Family(currentFt8x7Model());
}

bool currentIsFt857Family() {
  return currentFt8x7Model() == Ft8x7Model::Ft857;
}
