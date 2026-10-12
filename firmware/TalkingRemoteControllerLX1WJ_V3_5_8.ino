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
#include "radio_catalog.h"
#include "radio_monitor.h"
#include "radio_prefs.h"
#include "radio_profile.h"
#include "radio_protocol.h"
#include "radio_runtime.h"
#include "transport_serial.h"
#include "transport_usb_host.h"
#include "nanovna_client.h"
#include "engine_civ.h"
#include "engine_kenwood.h"
#include "ui_console.h"
#include "ui_keypad.h"
#include "ui_speech.h"
#include "debug_log.h"
#include "firmware_version.h"

static bool g_bootSpeakPending = false;
static uint32_t g_bootSpeakAtMs = 0;
static String g_serial0ServiceLine;

// A crash restart while the USB host was starting or running may have come from the USB host. Its
// profile then comes up without it, so the keypad can still pick another radio.
static void holdUsbHostIfCrashed(uint8_t slot) {
  const RadioProfile* profile = profileForSlot(slot);
  if (!profile || profile->link.port != RadioPort::UsbHost) return;
  switch (esp_reset_reason()) {
    case ESP_RST_PANIC:
    case ESP_RST_INT_WDT:
    case ESP_RST_TASK_WDT:
    case ESP_RST_WDT:
      usbHostHoldAfterCrash();
      break;
    default:
      break;
  }
}

static void waitForUsbSerial(uint32_t timeoutMs) {
  const uint32_t startMs = millis();
  while (!Serial && (millis() - startMs) < timeoutMs) {
    delay(10);
  }
}

void setup() {
  serialTransportDriveTxIdle(RS232_TX_PIN, false);

  Serial.begin(PC_BAUD);
  Serial0.begin(PC_BAUD);
  delay(200);
  waitForUsbSerial(2000);

  printProfileSlots();

  initSpeech();
  applyVolumeLevel(loadVolumeFromNvs(DEFAULT_VOLUME_LEVEL));
  applySpeechSpeed((SpeechSpeed)loadSpeechSpeedFromNvs((uint8_t)SpeechSpeed::Normal));

  g_profileId = loadProfileFromNvs(g_profileId);
  g_tuningSpeakEnabled = loadTuningSpeakFromNvs(true);
  g_verboseSpeech = loadVerboseFromNvs(true);
  holdUsbHostIfCrashed(g_profileId);
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
  initNanoVna();
  printHelp();

  // A USB radio is read when its serial port opens (pollRadioLinkEvents).
  if (currentProtocolType() != PROTO_YAESU_FT8X7 && currentConnectionProfile().port != RadioPort::UsbHost) {
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
  // After the boot announcement, so "usb on" follows the radio's name.
  if (!g_bootSpeakPending) pollRadioLinkEvents();

  pumpIncoming(CIV_PUMP_BUDGET_MS);
  pollFrequencyIfDue();
  speakPendingFreqIfIdle();
  pollSMeterIfDue();
  pollNanoVna();
  pollKeypadUi();

  String line = readLine();
  if (line.length()) processCommand(line);

  String serial0Line = readLineFrom(Serial0, g_serial0ServiceLine);
  if (serial0Line.length()) (void)processHamtrcServiceCommand(serial0Line, Serial0);
}
