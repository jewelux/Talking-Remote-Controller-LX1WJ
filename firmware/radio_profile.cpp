#include "radio_profile.h"

#include "radio_catalog.h"
#include "radio_prefs.h"
#include "radio_state.h"
#include "transport_serial.h"
#include "ui_speech.h"
#include "packet_ascii.h"
#include "protocol_yaesu_cat.h"

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

static const char* radioPortName(RadioPort port) {
  switch (port) {
    case RadioPort::CivJack: return "CI-V jack";
    case RadioPort::Rs232: return "RS-232";
    case RadioPort::CatTtl: return "CAT TTL";
  }
  return "?";
}

static const char* voiceVendorName(VoiceVendor vendor) {
  switch (vendor) {
    case VoiceVendor::Icom: return "icom";
    case VoiceVendor::Yaesu: return "yaesu";
    case VoiceVendor::Kenwood: return "kenwood";
    case VoiceVendor::Elecraft: return "elecraft";
    case VoiceVendor::Xiegu: return "xiegu";
  }
  return "?";
}

void printActiveProfileDetails() {
  const RadioProfile& sp = currentProfile();
  const ConnectionProfile& p = currentConnectionProfile();
  const SerialPortPins& pins = serialPortPins(p.port);

  Serial.println("[PROFILE DETAILS]");
  Serial.print("  slot: ");
  Serial.println((int)g_profileId);
  Serial.print("  name: ");
  Serial.println(sp.name);
  Serial.print("  protocol: ");
  Serial.println(protocolTypeToString(sp.protocol));
  Serial.print("  model: ");
  Serial.println(radioModelName(sp.model));
  if (sp.protocol == PROTO_CIV) {
    Serial.print("  civ_addr: 0x");
    Serial.println(g_civRadioAddr, HEX);
  }
  Serial.print("  port: ");
  Serial.println(radioPortName(p.port));
  Serial.print("  uart: ");
  Serial.println((int)pins.uartNum);
  Serial.print("  baud: ");
  Serial.println((unsigned long)p.baud);
  Serial.print("  bauds:");
  for (uint8_t i = 0; i < sp.link.bauds.count; ++i) {
    Serial.print(" ");
    Serial.print((unsigned long)sp.link.bauds.rates[i]);
  }
  Serial.println();
  Serial.print("  rx_pin: ");
  Serial.println((int)pins.rxPin);
  Serial.print("  tx_pin: ");
  Serial.println((int)pins.txPin);
  Serial.print("  tx_invert: ");
  Serial.println(pins.txInvert ? "1" : "0");
  Serial.print("  rx_invert: ");
  Serial.println(pins.rxInvert ? "1" : "0");

  Serial.print("  voice_vendor: ");
  Serial.println(voiceVendorName(sp.vendor));
  Serial.print("  voice_digits: ");
  Serial.println(sp.voiceDigits);
  Serial.print("  caps: freq=");
  Serial.print(sp.caps.getFreq ? "R" : "-");
  Serial.print(sp.caps.setFreq ? "W" : "-");
  Serial.print(" mode=");
  Serial.print(sp.caps.getMode ? "R" : "-");
  Serial.print(sp.caps.setMode ? "W" : "-");
  Serial.print(" smeter=");
  Serial.print(sp.caps.getSmeter ? "1" : "0");
  Serial.print(" power=");
  Serial.print(sp.caps.getPower ? "1" : "0");
  Serial.print(" rfpower=");
  Serial.print(sp.caps.getRfPower ? "R" : "-");
  Serial.print(sp.caps.setRfPower ? "W" : "-");
  Serial.print(" swr=");
  Serial.print(sp.caps.getSwr ? "1" : "0");
  Serial.print(" rxtx=");
  Serial.print(sp.caps.getRxTx ? "1" : "0");
  Serial.print(" txf=");
  Serial.print(sp.caps.getTxFreq ? "1" : "0");
  Serial.print(" nr=");
  Serial.print(sp.caps.getNr ? "R" : "-");
  Serial.print(sp.caps.setNr ? "W" : "-");
  Serial.print(" nrlvl=");
  Serial.print(sp.caps.getNrLevel ? "R" : "-");
  Serial.print(sp.caps.setNrLevel ? "W" : "-");
  Serial.print(" nb=");
  Serial.print(sp.caps.getNb ? "R" : "-");
  Serial.print(sp.caps.setNb ? "W" : "-");
  Serial.print(" nblvl=");
  Serial.print(sp.caps.getNbLevel ? "R" : "-");
  Serial.print(sp.caps.setNbLevel ? "W" : "-");
  Serial.print(" notch=");
  Serial.print(sp.caps.getNotch ? "R" : "-");
  Serial.print(sp.caps.setNotch ? "W" : "-");
  Serial.print(" pbt=");
  Serial.print(sp.caps.getPbtInner ? "1" : "0");
  Serial.print(sp.caps.getPbtOuter ? "1" : "0");
  Serial.print(" filter=");
  Serial.print(sp.caps.getFilterShape ? "S" : "-");
  Serial.print(sp.caps.getFilterWidth ? "W" : "-");
  Serial.print(" lock=");
  Serial.print(sp.caps.getDialLock ? "R" : "-");
  Serial.print(sp.caps.setDialLock ? "W" : "-");
  Serial.print(" mon=");
  Serial.print(sp.caps.getMonitor ? "R" : "-");
  Serial.print(sp.caps.setMonitor ? "W" : "-");
  Serial.print(" monlvl=");
  Serial.print(sp.caps.getMonitorLevel ? "R" : "-");
  Serial.print(sp.caps.setMonitorLevel ? "W" : "-");
  Serial.print(" xcv=");
  Serial.print(sp.caps.getTransceive ? "R" : "-");
  Serial.print(sp.caps.setTransceive ? "W" : "-");
  Serial.print(" tuner=");
  Serial.print(sp.caps.getTuner ? "R" : "-");
  Serial.print(sp.caps.setTuner ? "W" : "-");
  Serial.print(sp.caps.startTune ? "T" : "-");
  Serial.print(" vfo=");
  Serial.print(sp.caps.getVfo ? "R" : "-");
  Serial.print(sp.caps.setVfo ? "W" : "-");
  Serial.print(" vmode=");
  Serial.print(sp.caps.getVfoMode ? "R" : "-");
  Serial.print(sp.caps.setVfoMode ? "W" : "-");
  Serial.print(" split=");
  Serial.print(sp.caps.getSplit ? "R" : "-");
  Serial.print(sp.caps.setSplit ? "W" : "-");
  Serial.print(" rit=");
  Serial.print(sp.caps.getRit ? "R" : "-");
  Serial.print(sp.caps.setRit ? "W" : "-");
  Serial.print(" bstack=");
  Serial.print(sp.caps.getBandStack ? "1" : "0");
  Serial.println();
  if (g_experimentalCaps) Serial.println("  EXPERIMENTAL: all caps on");
}

void applyProfile(uint8_t slot) {
  const RadioProfile* profile = profileForSlot(slot);
  if (!profile) profile = profileForSlot(kDefaultProfileSlot);
  g_profileId = profile->slot;

  // The baud and CI-V address saved for this slot, if any; a baud the radio
  // does not offer falls back to its default.
  uint8_t civAddr = profile->link.civAddr;
  uint32_t baud = profile->link.baud;
  (void)loadConnectionOverrideFromNvs(g_profileId, civAddr, baud);
  if (!profile->link.bauds.contains(baud)) baud = profile->link.baud;
  selectActiveProfile(*profile, baud, civAddr);

  const ConnectionProfile& p = currentConnectionProfile();
  g_civRadioAddr = p.civAddr;
  serialTransportApplyProfile(p);
  resetLiveRadioState();
  g_yaesuCatTrace = false;
  if (currentProtocolType() == PROTO_YAESU_FT8X7) yaesuCatNoteLineOpened();
  if (currentProtocolType() == PROTO_ELECRAFT_ASCII) {
    delay(30);
    // Force documented default behavior so GET replies are not polluted by unsolicited auto-info.
    (void)asciiPacketSendCommand("AI0;");
    delay(10);
    (void)asciiPacketSendCommand("K30;");
    delay(10);
  }

  if (g_profileId != g_lastSavedProfile) {
    saveProfileToNvs(g_profileId);
    g_lastSavedProfile = g_profileId;
  }

  Serial.print("[PROFILE] Active: ");
  Serial.print(currentProfile().name);
  if (currentProtocolType() == PROTO_CIV) {
    Serial.print("  CI-V addr=0x");
    Serial.print(g_civRadioAddr, HEX);
  }
  Serial.print("  ");
  Serial.print(radioPortName(p.port));
  Serial.print("  baud=");
  Serial.println((unsigned long)p.baud);

  printActiveProfileDetails();
}

void speakCurrentProfile() {
  speakProfileIdentityFromSlot(g_profileId, false);
  speakValueOk();
}

const uint32_t kCivBaudRates[] = {4800, 9600, 19200, 38400, 57600, 115200};
const size_t kCivBaudRateCount = sizeof(kCivBaudRates) / sizeof(kCivBaudRates[0]);

bool setCurrentCivConnection(uint8_t civAddr, uint32_t baud) {
  if (currentProtocolType() != PROTO_CIV) return false;
  saveConnectionOverrideToNvs(g_profileId, civAddr, baud);
  applyProfile(g_profileId);
  return true;
}

void printProfileSlots() {
  Serial.println("[SLOTS]");
  for (size_t i = 0; i < profileCount(); ++i) {
    const RadioProfile& profile = profileAt(i);
    Serial.print("  ");
    Serial.print((int)profile.slot);
    Serial.print(" -> ");
    Serial.println(profile.name);
  }
}
