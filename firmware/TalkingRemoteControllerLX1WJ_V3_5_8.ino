/*
  Talking Remote Controller LX1WJ - ESP32-S3 + Keypad + Speech + Multi-Radio Profiles
  File: TalkingRemoteControllerLX1WJ_V3_5_8.ino
  Version: V3.5.8
  Bugfixes vs V3.5:
    - FT-817: yaesuCatSelectVfoA/B() waren Stubs (return false) -> VFO-Ops schlugen lautlos fehl
    - FT-817: Split-Status las Bit2 (ALC) statt Bit5 (Split) aus dem TX-Status-Byte
    - FT8x7: yaesuCatMemoryWrite() verwendete Opcode 0x0A (= CTCSS/DCS-Mode-Off) - kein Memory-Write
    - FT8x7: yaesuCatQueryAlcRaw() (0xBB=EEPROM-Read) las nur 1 von 2 Response-Bytes ->
             verursachte Buffer-Corruption bei nachfolgenden Frequenzabfragen
*/

#include "radio_globals.h"
#include "profile_loader.h"
#include "radio_catalog.h"
#include "radio_monitor.h"
#include "radio_prefs.h"
#include "radio_profile.h"
#include "radio_protocol.h"
#include "radio_runtime.h"
#include "sd_slots.h"
#include "engine_civ.h"
#include "engine_kenwood.h"
#include "ui_console.h"
#include "ui_keypad.h"
#include "ui_speech.h"
#include "debug_log.h"
#include "firmware_version.h"

static bool g_bootSpeakPending = false;
static uint32_t g_bootSpeakAtMs = 0;
static bool g_sdProfilesLoaded = false;
static String g_serial0ServiceLine;

static void waitForUsbSerial(uint32_t timeoutMs) {
  const uint32_t startMs = millis();
  while (!Serial && (millis() - startMs) < timeoutMs) {
    delay(10);
  }
}

static void printSdBootSummary() {
  Serial.print("[BOOT] SD profile loading: ");
  Serial.println(getLastSdLoadStatusText());
  if (!g_sdProfilesLoaded) {
    Serial.println("[BOOT] Using LIGHT Icom fallback slots (Bank 9 keys 1-9)");
  }
}

void setup() {
  digitalWrite(RS232_TX_PIN, HIGH);
  pinMode(RS232_TX_PIN, OUTPUT);

  Serial.begin(PC_BAUD);
  Serial0.begin(PC_BAUD);
  delay(200);
  waitForUsbSerial(2000);

  seedBuiltInSlots();
  g_sdProfilesLoaded = loadProfilesFromSd();
  printProfileSlots();
  printSdBootSummary();

  initSpeech();
  applyVolumeLevel(loadVolumeFromNvs(DEFAULT_VOLUME_LEVEL));

  g_profileId = loadProfileFromNvs(g_profileId);
  g_tuningSpeakEnabled = loadTuningSpeakFromNvs(true);
  applyProfile(g_profileId);

  DBG_PRINT("Talking Remote Controller LX1WJ (");
  DBG_PRINT(HAMTRC_FIRMWARE_VERSION);
  DBG_PRINT(") - Selected radio: ");
  DBG_PRINT(currentProfile().name);
  DBG_PRINT("  CI-V addr=0x");
  if (debugLogEnabled()) Serial.println(g_civRadioAddr, HEX);

  g_bootSpeakPending = true;
  g_bootSpeakAtMs = millis() + 900;

  initKeypadUi();
  printHelp();
  printSdBootSummary();

  if (currentProtocolType() != PROTO_YAESU_FT8X7) {
    (void)refreshLiveFrequency();
    (void)refreshLiveMode();
  }
}

void loop() {
  pollKeypadUi();

  if (g_bootSpeakPending && (int32_t)(millis() - g_bootSpeakAtMs) >= 0) {
    g_bootSpeakPending = false;
    speakBootProfile();
  }

  pumpIncoming(CIV_PUMP_BUDGET_MS);
  pollFrequencyIfDue();
  speakPendingFreqIfIdle();
  pollSMeterIfDue();
  pollKeypadUi();

  String line = readLine();
  if (line.length()) processCommand(line);

  String serial0Line = readLineFrom(Serial0, g_serial0ServiceLine);
  if (serial0Line.length()) (void)processHamtrcServiceCommand(serial0Line, Serial0);
}
