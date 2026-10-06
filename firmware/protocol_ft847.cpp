#include "protocol_ft847.h"

#include "protocol_ascii.h"
#include "protocol_yaesu_cat.h"
#include "transport_serial.h"

// Minimum gap after a frame before the next one may start.
static constexpr uint32_t FT847_MIN_COMMAND_GAP_MS = 20;

static uint32_t s_byteGapMs = FT847_DEFAULT_BYTE_GAP_MS;
// Earliest time the next frame may be sent.
static uint32_t s_nextTxAllowedMs = 0;
// CAT ON goes out before the next frame.
static bool s_catOnPending = true;
// Set by CAT OFF from the console: nothing is sent until CAT ON.
static bool s_catHeldOff = false;
// Set by F847POLL OFF from the console.
static bool s_backgroundPollPaused = false;

bool ft847BackgroundPollPaused() { return s_backgroundPollPaused; }

void ft847SetBackgroundPollPaused(bool paused) { s_backgroundPollPaused = paused; }

static void ft847Trace(const char* label, const uint8_t frame[5]) {
  if (!g_yaesuCatTrace || !Serial) return;
  Serial.print("[F847] ");
  Serial.print(millis());
  Serial.print(" ms ");
  Serial.print(label);
  Serial.print(": ");
  yaesuCatPrintFrame(frame);
  Serial.println();
}

// Writes one frame byte by byte with the configured gap, keeping the gap to the previous frame.
static void ft847WriteFrame(const uint8_t frame[5]) {
  const int32_t waitMs = (int32_t)(s_nextTxAllowedMs - millis());
  if (waitMs > 0) delay((uint32_t)waitMs);
  ft847Trace("TX", frame);
  serialTransportSetFt847WriteGate(true);
  for (int i = 0; i < 5; ++i) {
    serialTransportWriteByte(frame[i]);
    serialTransportFlushOutput();
    if (i < 4 && s_byteGapMs) delay(s_byteGapMs);
  }
  serialTransportSetFt847WriteGate(false);
  s_nextTxAllowedMs = millis() + FT847_MIN_COMMAND_GAP_MS;
}

// Sends CAT ON first when it is due. False while CAT is held off from the console.
static bool ft847EnsureCatOn() {
  if (s_catHeldOff) return false;
  if (!s_catOnPending) return true;
  uint8_t frame[5];
  ft847BuildFrame(FT847_OP_CAT_ON, frame);
  yaesuCatFlushInput();
  ft847WriteFrame(frame);
  delay(FT847_WRITE_SETTLE_MS);
  s_catOnPending = false;
  return true;
}

// The radio may have been switched off and on, which turns CAT off: send CAT ON again before
// the next frame.
static void ft847NoteNoReply() { s_catOnPending = true; }

static bool ft847SendWriteOnly(const uint8_t frame[5]) {
  if (!ft847EnsureCatOn()) return false;
  yaesuCatFlushInput();
  ft847WriteFrame(frame);
  delay(FT847_WRITE_SETTLE_MS);
  return true;
}

static bool ft847Transact1(const uint8_t frame[5], uint8_t& rsp, uint32_t timeoutMs) {
  if (!ft847EnsureCatOn()) return false;
  yaesuCatFlushInput();
  ft847WriteFrame(frame);
  if (yaesuCatRead1(rsp, timeoutMs)) return true;
  ft847NoteNoReply();
  return false;
}

static bool ft847Transact5(const uint8_t frame[5], uint8_t rsp[5], uint32_t timeoutMs) {
  if (!ft847EnsureCatOn()) return false;
  yaesuCatFlushInput();
  ft847WriteFrame(frame);
  if (yaesuCatRead5(rsp, timeoutMs)) return true;
  ft847NoteNoReply();
  return false;
}

void ft847NoteLineOpened() {
  s_nextTxAllowedMs = millis() + FT847_LINE_OPEN_HOLDOFF_MS;
  s_catOnPending = true;
}

uint32_t ft847ByteGapMs() { return s_byteGapMs; }

void ft847SetByteGapMs(uint32_t ms) { s_byteGapMs = ms > 200 ? 200 : ms; }

void ft847CatOn() {
  s_catHeldOff = false;
  s_catOnPending = true;
  (void)ft847EnsureCatOn();
}

void ft847CatOff() {
  uint8_t frame[5];
  ft847BuildFrame(FT847_OP_CAT_OFF, frame);
  yaesuCatFlushInput();
  ft847WriteFrame(frame);
  delay(FT847_WRITE_SETTLE_MS);
  s_catHeldOff = true;
}

bool ft847CatHeldOff() { return s_catHeldOff; }

void ft847SendRaw(const uint8_t frame[5]) { (void)ft847SendWriteOnly(frame); }

bool ft847QueryRaw1(const uint8_t frame[5], uint8_t& rsp, uint32_t timeoutMs) {
  return ft847Transact1(frame, rsp, timeoutMs);
}

bool ft847QueryRaw5(const uint8_t frame[5], uint8_t rsp[5], uint32_t timeoutMs) {
  return ft847Transact5(frame, rsp, timeoutMs);
}

// Reads the main VFO's frequency and mode frame and checks that it is aligned.
static bool ft847ReadFreqModeFrame(uint8_t rsp[5], uint32_t timeoutMs) {
  uint8_t frame[5];
  ft847BuildFrame(FT847_OP_READ_FREQ_MODE, frame);
  if (!ft847Transact5(frame, rsp, timeoutMs)) return false;
  if (!ft847FreqFieldValid(rsp) || !ft847ModeBaseKnown(ft847ModeBase(rsp[4]))) {
    yaesuCatMarkLineDirty();
    return false;
  }
  return true;
}

bool ft847QueryFrequency(const RadioProfile& sp, uint64_t& hzOut, uint32_t timeoutMs) {
  if (!sp.caps.getFreq) return false;
  uint8_t rsp[5] = {0};
  if (!ft847ReadFreqModeFrame(rsp, timeoutMs)) return false;
  hzOut = ft847DecodeFreqHz(rsp);
  return true;
}

bool ft847SetFrequency(const RadioProfile& sp, uint64_t hz) {
  if (!sp.caps.setFreq) return false;
  if (hz < FT847_MIN_FREQ_HZ || hz > FT847_MAX_FREQ_HZ) return false;
  uint8_t frame[5];
  ft847BuildSetFreqFrame(hz, frame);
  if (!ft847SendWriteOnly(frame)) return false;
  delay(FT847_FREQ_MODE_SETTLE_MS);
  return true;
}

bool ft847QueryModeByte(uint8_t& modeByteOut, uint32_t timeoutMs) {
  uint8_t rsp[5] = {0};
  if (!ft847ReadFreqModeFrame(rsp, timeoutMs)) return false;
  modeByteOut = rsp[4];
  return true;
}

bool ft847QueryMode(const RadioProfile& sp, uint8_t& modeOut, uint32_t timeoutMs) {
  if (!sp.caps.getMode) return false;
  uint8_t modeByte = 0;
  if (!ft847QueryModeByte(modeByte, timeoutMs)) return false;
  // The profile's [modes] list the wide codes; a narrow filter reads as its mode.
  return profileInternalModeForCode(sp, byteToUpperHex(ft847ModeBase(modeByte)), modeOut);
}

bool ft847SetMode(const RadioProfile& sp, uint8_t mode) {
  if (!sp.caps.setMode) return false;
  String code;
  uint8_t modeByte = 0;
  if (!profileModeCodeForInternal(sp, mode, code)) return false;
  if (!parseHexByteString(code, modeByte)) return false;
  if (!ft847ModeBaseKnown(ft847ModeBase(modeByte))) return false;
  uint8_t frame[5];
  ft847BuildSetModeFrame(modeByte, frame);
  if (!ft847SendWriteOnly(frame)) return false;
  delay(FT847_FREQ_MODE_SETTLE_MS);
  return true;
}

bool ft847QueryRxStatus(uint8_t& rxStatusOut, uint32_t timeoutMs) {
  uint8_t frame[5];
  ft847BuildFrame(FT847_OP_READ_RX_STATUS, frame);
  return ft847Transact1(frame, rxStatusOut, timeoutMs);
}

bool ft847QueryTxStatus(uint8_t& txStatusOut, uint32_t timeoutMs) {
  uint8_t frame[5];
  ft847BuildFrame(FT847_OP_READ_TX_STATUS, frame);
  return ft847Transact1(frame, txStatusOut, timeoutMs);
}

bool ft847QuerySMeterRaw(const RadioProfile& sp, int32_t& rawOut, uint32_t timeoutMs) {
  if (!sp.caps.getSmeter) return false;
  uint8_t rxStatus = 0;
  if (!ft847QueryRxStatus(rxStatus, timeoutMs)) return false;
  rawOut = rxStatus;
  return true;
}

SMeterReading ft847SMeterFromRaw(uint8_t rxStatus) {
  SMeterReading reading;
  ft847DecodeSMeter(rxStatus, reading.sUnits, reading.dbOverS9);
  return reading;
}

bool ft847QueryRxTx(const RadioProfile& sp, bool& txOut, uint32_t timeoutMs) {
  if (!sp.caps.getRxTx) return false;
  uint8_t txStatus = 0;
  if (!ft847QueryTxStatus(txStatus, timeoutMs)) return false;
  txOut = ft847TxStatusTransmitting(txStatus);
  return true;
}
