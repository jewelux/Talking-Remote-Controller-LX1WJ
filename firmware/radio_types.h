#pragma once

#include <Arduino.h>
#include <Preferences.h>
#include "driver/uart.h"
#include <Keypad.h>
#include <SPI.h>
#include <SD.h>

extern "C" {
#include "driver/i2s.h"
}

#include "config_pins.h"
#include "voice_data.h"

struct VoiceClip {
  const char* name;
  const uint8_t* data;
  size_t len;
};

static constexpr bool AUTO_SEND_BANK1_QUERIES = true;
static constexpr uint8_t MAX_PROFILE_SLOTS = 24;
static constexpr uint8_t CIV_MY_ADDR = 0xE0;
static constexpr uint8_t CIV_CTRL_ADDR = CIV_MY_ADDR;
static constexpr uint32_t CIV_PUMP_BUDGET_MS = 3;

static const bool FREQ_SPEAK_START_IMMEDIATELY = false;
static const uint32_t FREQ_SPEAK_IDLE_MS = 1500;
static const bool FREQ_POLL_ENABLE = true;
static const uint32_t FREQ_POLL_MS = 400;
static const uint32_t FREQ_POLL_TIMEOUT_MS = 80;
static const uint32_t FREQ_SPEAK_MIN_STEP_HZ = 10;
static const uint32_t FREQ_SPEAK_MIN_INTERVAL_MS = 5000;

static const bool SMETER_POLL_ENABLE = false;
static const uint32_t SMETER_POLL_MS = 350;
static const uint32_t SMETER_POLL_TIMEOUT_MS = 80;
static const bool SMETER_SPEAK_ENABLE = false;
static const uint8_t SMETER_SPEAK_MIN_DELTA_S = 1;
static const uint32_t SMETER_SPEAK_MIN_INTERVAL_MS = 2500;
static const int32_t SMETER_RAW_AT_S0 = 21;
static const int32_t SMETER_RAW_AT_S9 = 297;

enum IcomModel : uint8_t {
  ICOM_IC_7300,
  ICOM_IC_706MKIIG
};

static constexpr IcomModel ICOM_MODEL = ICOM_IC_7300;

enum ProtocolType : uint8_t {
  PROTO_CIV = 0,
  PROTO_KENWOOD_ASCII = 1,
  PROTO_ELECRAFT_ASCII = 2,
  PROTO_YAESU_FT8X7 = 3,
  PROTO_YAESU_FTDX_ASCII = 4
};

struct CivProfile {
  uint8_t civAddr;
  const char* name;
  uint32_t baud;
  int8_t uartNum;
  int8_t rxPin;
  int8_t txPin;
  bool txInvert;
  bool rxInvert;
};

struct RadioCapabilities {
  bool getFreq;
  bool setFreq;
  bool getMode;
  bool setMode;
  bool getSmeter;
  bool getPower;
  bool getRfPower;
  bool setRfPower;
  bool getSwr;
  bool getRxTx;
  bool getTxFreq;
  bool getNr;
  bool setNr;
  bool getNrLevel;
  bool setNrLevel;
  bool getNb;
  bool setNb;
  bool getNbLevel;
  bool setNbLevel;
  bool getNotch;
  bool setNotch;
  bool getNotchWidth;
  bool setNotchWidth;
  bool getPbtInner;
  bool setPbtInner;
  bool getPbtOuter;
  bool setPbtOuter;
  bool getFilterShape;
  bool setFilterShape;
  bool getFilterWidth;
  bool setFilterWidth;
  bool getDialLock;
  bool setDialLock;
  bool getMonitor;
  bool setMonitor;
  bool getMonitorLevel;
  bool setMonitorLevel;
  bool getTransceive;
  bool setTransceive;
  bool getTuner;
  bool setTuner;
  bool startTune;
  bool getVfo;
  bool setVfo;
  bool getVfoMode;
  bool setVfoMode;
  bool getSplit;
  bool setSplit;
  bool getRit;
  bool setRit;
  bool getBandStack;
};

struct AsciiCommandProfile {
  char freqGet[20];
  char freqSetFormat[32];
  char modeGet[20];
  char modeSetFormat[20];
  char vfoAGet[20];
  char vfoASetFormat[32];
  char vfoBGet[20];
  char vfoBSetFormat[32];
  char ifGet[20];
  char idGet[20];
  char omGet[20];
  char smeterGet[20];
  char powerGet[20];
  char swrGet[20];
  char nrGet[20];
  char nrOnCmd[20];
  char nrOffCmd[20];
  char nbGet[20];
  char nbOnCmd[20];
  char nbOffCmd[20];
  char preampGet[20];
  char preampOnCmd[20];
  char preampOffCmd[20];
  char agcGet[20];
  char agcFastCmd[20];
  char agcSlowCmd[20];
  char agcOffCmd[20];
  char powerStateGet[20];
  char powerStateOnCmd[20];
  char powerStateOffCmd[20];
  char tunerGet[20];
  char tunerOnCmd[20];
  char tunerOffCmd[20];
  char tuneStartCmd[20];
  char splitGet[20];
  char splitOnCmd[20];
  char splitOffCmd[20];
  char vfoGet[20];
  char vfoACmd[20];
  char vfoBCmd[20];
  char vfoSwapCmd[20];
  char notchGet[20];
  char notchOnCmd[20];
  char notchOffCmd[20];
  char lockGet[20];
  char lockOnCmd[20];
  char lockOffCmd[20];
  char freqReplyPrefix[8];
  char modeReplyPrefix[8];
  char ifReplyPrefix[8];
  char idReplyPrefix[8];
  char omReplyPrefix[8];
  char smeterReplyPrefix[8];
  char powerReplyPrefix[8];
  char swrReplyPrefix[8];
  char nrReplyPrefix[8];
  char nbReplyPrefix[8];
  char preampReplyPrefix[8];
  char agcReplyPrefix[8];
  char powerStateReplyPrefix[8];
  char tunerReplyPrefix[8];
  char splitReplyPrefix[8];
  char vfoReplyPrefix[8];
  char notchReplyPrefix[8];
  char lockReplyPrefix[8];
  char modeLsb[4];
  char modeUsb[4];
  char modeAm[4];
  char modeCw[4];
  char modeRtty[4];
  char modeFm[4];
  char modeCwr[4];
  char modeRttyR[4];
  char modeDigi[4];
};

struct Ft8x7Bank6Profile {
  uint32_t repeaterOffsetsHz[2];
  uint16_t ctcssDefaultTenths;
  uint16_t dcsDefaultCode;
};

enum NotchWidth : uint8_t {
  NOTCH_WIDTH_NAR = 0,
  NOTCH_WIDTH_MID = 1,
  NOTCH_WIDTH_WIDE = 2,
  NOTCH_WIDTH_UNKNOWN = 0xFF
};

enum ProfileId : uint8_t {
  PROFILE_ID_SLOT1 = 1,
  PROFILE_ID_SLOT2 = 2,
  PROFILE_ID_SLOT3 = 3,
  PROFILE_ID_SLOT4 = 4,
  PROFILE_ID_SLOT5 = 5,
  PROFILE_ID_SLOT6 = 6,
  PROFILE_ID_SLOT7 = 7,
  PROFILE_ID_SLOT8 = 8,
  PROFILE_ID_SLOT9 = 9,
  PROFILE_ID_SLOT10 = 10,
  PROFILE_ID_SLOT11 = 11,
  PROFILE_ID_SLOT12 = 12,
  PROFILE_ID_SLOT13 = 13,
  PROFILE_ID_SLOT14 = 14,
  PROFILE_ID_SLOT15 = 15,
  PROFILE_ID_SLOT16 = 16,
  PROFILE_ID_SLOT17 = 17,
  PROFILE_ID_SLOT18 = 18,
  PROFILE_ID_SLOT19 = 19,
  PROFILE_ID_SLOT20 = 20,
  PROFILE_ID_SLOT21 = 21,
  PROFILE_ID_SLOT22 = 22,
  PROFILE_ID_SLOT23 = 23,
  PROFILE_ID_SLOT24 = 24
};

static constexpr uint8_t PROFILE_ID_7300 = PROFILE_ID_SLOT1;
static constexpr uint8_t PROFILE_ID_7300_RS232 = PROFILE_ID_SLOT2;
static constexpr uint8_t PROFILE_ID_706_CIV = PROFILE_ID_SLOT3;
static constexpr uint8_t PROFILE_ID_706_RS232 = PROFILE_ID_SLOT4;
static constexpr uint8_t PROFILE_ID_705 = PROFILE_ID_SLOT5;
static constexpr uint8_t PROFILE_ID_7760 = PROFILE_ID_SLOT6;

struct StoredProfile {
  CivProfile civ;
  ProtocolType protocolType;
  RadioCapabilities caps;
  AsciiCommandProfile ascii;
  Ft8x7Bank6Profile ft8x7Bank6;
  uint16_t rfPowerMaxWatts;
  bool valid;
  bool fromSd;
  char name[32];
  char voiceVendor[16];
  char voiceDigits[16];
  char variant[16];
};

struct CivDecoded {
  bool ok = false;
  uint8_t to = 0;
  uint8_t from = 0;
  uint8_t cmd = 0;
  const uint8_t* payload = nullptr;
  size_t payloadLen = 0;
};

struct BandStackEntry {
  uint8_t bandCode = 0;
  uint8_t registerCode = 0;
  uint64_t freqHz = 0;
  uint8_t mode = 0xFF;
  uint8_t filter = 0xFF;
};

struct LiveState {
  bool freqValid = false;
  uint64_t freqHz = 0;
  uint32_t lastFreqMs = 0;
  uint32_t lastFreqPollMs = 0;
  bool tuning = false;
  uint64_t pendingHz = 0;
  uint64_t tuningStartSpokenHz = 0;
  uint32_t lastChangeMs = 0;
  uint64_t lastSpokenHz = 0;
  uint32_t lastSpokenMs = 0;
  bool modeValid = false;
  uint8_t mode = 0xFF;
  uint32_t lastModeMs = 0;
  bool smValid = false;
  int32_t smRaw = 0;
  uint8_t smS = 0;
  uint32_t lastSmMs = 0;
  uint8_t lastSpokenS = 0xFF;
  uint32_t lastSmPollMs = 0;
  uint32_t lastSmSpokenMs = 0;
  bool powerValid = false;
  int32_t powerRaw = 0;
  uint32_t lastPowerMs = 0;
  bool swrValid = false;
  int32_t swrRaw = 0;
  uint32_t lastSwrMs = 0;
  bool nrValid = false;
  bool nrOn = false;
  uint32_t lastNrMs = 0;
  bool nbValid = false;
  bool nbOn = false;
  uint32_t lastNbMs = 0;
  bool notchValid = false;
  bool notchOn = false;
  bool notchWidthValid = false;
  NotchWidth notchWidth = NOTCH_WIDTH_UNKNOWN;
  uint32_t lastNotchMs = 0;
  bool ctcssValid = false;
  uint16_t ctcssTenths = 0;
  bool dcsValid = false;
  uint16_t dcsCode = 0;
  bool activeVfoKnown = false;
  bool activeVfoA = true;
  bool splitKnown = false;
  bool splitOn = false;
  bool lockKnown = false;
  bool lockOn = false;
};
