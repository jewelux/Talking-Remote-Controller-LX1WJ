#include "radio_profile.h"

#include "radio_catalog.h"
#include "radio_prefs.h"
#include "radio_state.h"
#include "transport_serial.h"
#include "ui_speech.h"
#include "packet_ascii.h"

const char* protocolTypeToString(ProtocolType pt) {
  switch (pt) {
    case PROTO_CIV: return "CI-V";
    case PROTO_KENWOOD_ASCII: return "KENWOOD_ASCII";
    case PROTO_ELECRAFT_ASCII: return "ELECRAFT_ASCII";
    case PROTO_YAESU_FT8X7: return "YAESU_FT8X7_CAT";
    case PROTO_YAESU_FTDX_ASCII: return "YAESU_FTDX_ASCII_CAT";
    default: return "UNKNOWN";
  }
}

void printActiveProfileDetails() {
  const StoredProfile* sp = storedProfileForId(g_profileId);
  const CivProfile& p = currentProfile();

  Serial.println("[PROFILE DETAILS]");
  Serial.print("  slot: ");
  Serial.println((int)g_profileId);
  Serial.print("  name: ");
  Serial.println(p.name ? p.name : "(null)");
  Serial.print("  source: ");
  Serial.println((sp && sp->fromSd) ? "SD" : "built-in");
  Serial.print("  protocol: ");
  Serial.println(protocolTypeToString(currentProtocolType()));
  if (currentProtocolType() == PROTO_CIV) {
    Serial.print("  civ_addr: 0x");
    Serial.println(g_civRadioAddr, HEX);
  }
  Serial.print("  uart: ");
  Serial.println((int)p.uartNum);
  Serial.print("  baud: ");
  Serial.println((unsigned long)p.baud);
  Serial.print("  rx_pin: ");
  Serial.println((int)p.rxPin);
  Serial.print("  tx_pin: ");
  Serial.println((int)p.txPin);
  Serial.print("  tx_invert: ");
  Serial.println(p.txInvert ? "1" : "0");
  Serial.print("  rx_invert: ");
  Serial.println(p.rxInvert ? "1" : "0");

  if (sp) {
    Serial.print("  voice_vendor: ");
    Serial.println(sp->voiceVendor);
    Serial.print("  voice_digits: ");
    Serial.println(sp->voiceDigits);
    Serial.print("  caps: freq=");
    Serial.print(sp->caps.getFreq ? "R" : "-");
    Serial.print(sp->caps.setFreq ? "W" : "-");
    Serial.print(" mode=");
    Serial.print(sp->caps.getMode ? "R" : "-");
    Serial.print(sp->caps.setMode ? "W" : "-");
    Serial.print(" smeter=");
    Serial.print(sp->caps.getSmeter ? "1" : "0");
    Serial.print(" power=");
    Serial.print(sp->caps.getPower ? "1" : "0");
    Serial.print(" swr=");
    Serial.print(sp->caps.getSwr ? "1" : "0");
    Serial.print(" nr=");
    Serial.print(sp->caps.getNr ? "R" : "-");
    Serial.print(sp->caps.setNr ? "W" : "-");
    Serial.print(" nb=");
    Serial.print(sp->caps.getNb ? "R" : "-");
    Serial.print(sp->caps.setNb ? "W" : "-");
    Serial.print(" notch=");
    Serial.print(sp->caps.getNotch ? "R" : "-");
    Serial.print(sp->caps.setNotch ? "W" : "-");
    Serial.println();
  }
}

void applyProfile(uint8_t profileId) {
  if (!isValidProfileId(profileId) || !storedProfileForId(profileId)) {
    profileId = PROFILE_ID_7300;
  }
  g_profileId = profileId;

  const CivProfile& p = currentProfile();
  g_civRadioAddr = p.civAddr;
  serialTransportApplyProfile(p);
  if (currentProtocolType() == PROTO_ELECRAFT_ASCII) {
    delay(30);
    // Force documented default behavior so GET replies are not polluted by unsolicited auto-info.
    (void)asciiPacketSendCommand("AI0;");
    delay(10);
    (void)asciiPacketSendCommand("K30;");
    delay(10);
  }
  resetLiveRadioState();

  if (g_profileId != g_lastSavedProfile) {
    saveProfileToNvs(g_profileId);
    g_lastSavedProfile = g_profileId;
  }

  Serial.print("[PROFILE] Active: ");
  Serial.print(p.name);
  if (currentProtocolType() == PROTO_CIV) {
    Serial.print("  CI-V addr=0x");
    Serial.print(g_civRadioAddr, HEX);
  }
  Serial.print("  UART");
  Serial.print(p.uartNum);
  Serial.print("  baud=");
  Serial.print(p.baud);
  Serial.print("  RX=");
  Serial.print(p.rxPin);
  Serial.print("  TX=");
  Serial.println(p.txPin);

  printActiveProfileDetails();
}

void speakCurrentProfile() {
  speakProfileIdentityFromSlot(g_profileId, true);
}
