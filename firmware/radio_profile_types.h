#pragma once

// What a radio is: the types of the built-in profile table (radio_profile_table.cpp).
// No Arduino.h, so the host tests can build the table.

#include <stddef.h>
#include <stdint.h>

enum ProtocolType : uint8_t {
  PROTO_CIV = 0,
  PROTO_KENWOOD_ASCII = 1,
  PROTO_ELECRAFT_ASCII = 2,
  PROTO_YAESU_FT8X7 = 3,
  PROTO_YAESU_FTDX_ASCII = 4,
  PROTO_YAESU_FT847 = 5  // 5-byte CAT like the FT-8x7, but its own opcodes (protocol_ft847.*)
};

// Only bool members, all off unless a profile sets them: EXPERIMENTAL ON sets
// them all through a bool array (radio_catalog.cpp).
struct RadioCapabilities {
  bool getFreq = false;
  bool setFreq = false;
  bool getMode = false;
  bool setMode = false;
  bool getSmeter = false;
  bool getPower = false;
  bool getRfPower = false;
  bool setRfPower = false;
  bool getSwr = false;
  bool getRxTx = false;
  bool getTxFreq = false;
  bool getNr = false;
  bool setNr = false;
  bool getNrLevel = false;
  bool setNrLevel = false;
  bool getNb = false;
  bool setNb = false;
  bool getNbLevel = false;
  bool setNbLevel = false;
  bool getNotch = false;
  bool setNotch = false;
  bool getNotchWidth = false;
  bool setNotchWidth = false;
  bool getPbtInner = false;
  bool setPbtInner = false;
  bool getPbtOuter = false;
  bool setPbtOuter = false;
  bool getFilterShape = false;
  bool setFilterShape = false;
  bool getFilterWidth = false;
  bool setFilterWidth = false;
  bool getDialLock = false;
  bool setDialLock = false;
  bool getMonitor = false;
  bool setMonitor = false;
  bool getMonitorLevel = false;
  bool setMonitorLevel = false;
  bool getTransceive = false;
  bool setTransceive = false;
  bool getTuner = false;
  bool setTuner = false;
  bool startTune = false;
  bool getVfo = false;
  bool setVfo = false;
  bool getVfoMode = false;
  bool setVfoMode = false;
  bool getSplit = false;
  bool setSplit = false;
  bool getRit = false;
  bool setRit = false;
  bool getBandStack = false;
};

// The controller's serial connector the radio is wired to. Its UART, pins and
// line inversion are in transport_serial.cpp.
enum class RadioPort : uint8_t {
  CivJack,  // Icom CI-V jack
  Rs232,    // RS-232 level shifter
  CatTtl,   // TTL CAT header
};

// The baud rates the radio's CAT or CI-V port can be set to.
struct BaudRates {
  const uint32_t* rates = nullptr;
  uint8_t count = 0;

  constexpr BaudRates() = default;
  template <size_t N>
  constexpr BaudRates(const uint32_t (&list)[N]) : rates(list), count((uint8_t)N) {}

  constexpr bool contains(uint32_t baud) const {
    for (uint8_t i = 0; i < count; ++i) {
      if (rates[i] == baud) return true;
    }
    return false;
  }
};

// How the radio is connected out of the box. The user can pick another baud
// from bauds and, on CI-V, another address; radio_prefs saves them per slot.
struct LinkDefaults {
  RadioPort port = RadioPort::Rs232;
  uint32_t baud = 0;
  BaudRates bauds{};
  uint8_t civAddr = 0;  // CI-V radios only
};

// Spoken before the profile's voice digits.
enum class VoiceVendor : uint8_t { Icom, Yaesu, Kenwood, Elecraft, Xiegu };

// The radio, where code has a path for that model alone. Every other radio is
// Generic, and its profile says all there is to know about it.
enum class RadioModel : uint8_t {
  Generic,
  Ic7760,  // CI-V main/sub receiver selection
  Ts480,   // two-level NR
  Ft817,   // FT-8x7 EEPROM map, VFO tracking and keypad layout
  Ft818,
  Ft857,   // FT-857 and FT-897
  Ftdx10,  // FTDX10 keypad layout and hidden commands
};

// The commands of an ASCII CAT radio (Kenwood, Elecraft, Yaesu FTDX), and the
// prefixes of their replies. An empty string is a command the radio lacks.
struct AsciiCommandSet {
  const char* freqGet = "";
  const char* freqSetFormat = "";
  const char* modeGet = "";
  const char* modeSetFormat = "";
  const char* vfoAGet = "";
  const char* vfoASetFormat = "";
  const char* vfoBGet = "";
  const char* vfoBSetFormat = "";
  const char* ifGet = "";
  const char* idGet = "";
  const char* omGet = "";
  const char* smeterGet = "";
  const char* powerGet = "";
  const char* swrGet = "";
  const char* nrGet = "";
  const char* nrOnCmd = "";
  const char* nrOffCmd = "";
  const char* nbGet = "";
  const char* nbOnCmd = "";
  const char* nbOffCmd = "";
  const char* preampGet = "";
  const char* preampOnCmd = "";
  const char* preampOffCmd = "";
  const char* agcGet = "";
  const char* agcFastCmd = "";
  const char* agcSlowCmd = "";
  const char* agcOffCmd = "";
  const char* powerStateGet = "";
  const char* powerStateOnCmd = "";
  const char* powerStateOffCmd = "";
  const char* tunerGet = "";
  const char* tunerOnCmd = "";
  const char* tunerOffCmd = "";
  const char* tuneStartCmd = "";
  const char* splitGet = "";
  const char* splitOnCmd = "";
  const char* splitOffCmd = "";
  const char* vfoGet = "";
  const char* vfoACmd = "";
  const char* vfoBCmd = "";
  const char* vfoSwapCmd = "";
  const char* notchGet = "";
  const char* notchOnCmd = "";
  const char* notchOffCmd = "";
  const char* lockGet = "";
  const char* lockOnCmd = "";
  const char* lockOffCmd = "";
  const char* freqReplyPrefix = "";
  const char* modeReplyPrefix = "";
  const char* ifReplyPrefix = "";
  const char* idReplyPrefix = "";
  const char* omReplyPrefix = "";
  const char* smeterReplyPrefix = "";
  const char* powerReplyPrefix = "";
  const char* swrReplyPrefix = "";
  const char* nrReplyPrefix = "";
  const char* nbReplyPrefix = "";
  const char* preampReplyPrefix = "";
  const char* agcReplyPrefix = "";
  const char* powerStateReplyPrefix = "";
  const char* tunerReplyPrefix = "";
  const char* splitReplyPrefix = "";
  const char* vfoReplyPrefix = "";
  const char* notchReplyPrefix = "";
  const char* lockReplyPrefix = "";
};

// The radio's code for each mode (ASCII and FT-8x7 radios). An empty string is
// a mode the radio lacks: it is left out of mode select and MODE LIST.
struct ModeCodes {
  const char* lsb = "";
  const char* usb = "";
  const char* am = "";
  const char* cw = "";
  const char* rtty = "";
  const char* fm = "";
  const char* cwr = "";
  const char* rttyR = "";
  const char* digi = "";
  const char* pkt = "";
};

// FT-8x7 Bank 6 presets: repeater offsets, and the tone and DCS code offered
// before one has been set.
struct Ft8x7Bank6Profile {
  uint32_t repeaterOffsetsHz[2] = {600000UL, 7600000UL};
  uint16_t ctcssDefaultTenths = 885;
  uint16_t dcsDefaultCode = 23;
};

inline constexpr AsciiCommandSet kNoAsciiCommands{};
inline constexpr ModeCodes kNoModeCodes{};

// One radio in one slot.
struct RadioProfile {
  uint8_t slot = 0;          // the number the user picks it by
  const char* name = "";     // shown and logged
  VoiceVendor vendor = VoiceVendor::Icom;
  const char* voiceDigits = "";  // spoken digit by digit after the vendor
  RadioModel model = RadioModel::Generic;
  ProtocolType protocol = PROTO_CIV;
  LinkDefaults link{};
  RadioCapabilities caps{};
  const AsciiCommandSet* commands = &kNoAsciiCommands;
  const ModeCodes* modes = &kNoModeCodes;
  uint16_t rfPowerMaxWatts = 100;
  Ft8x7Bank6Profile ft8x7Bank6{};
};
