#include "ft847_codec.h"

void ft847BuildFrame(uint8_t opcode, uint8_t frame[5]) {
  frame[0] = 0x00;
  frame[1] = 0x00;
  frame[2] = 0x00;
  frame[3] = 0x00;
  frame[4] = opcode;
}

uint64_t ft847DecodeFreqHz(const uint8_t data[4]) {
  uint64_t digits = 0;
  for (int i = 0; i < 4; ++i) {
    digits = digits * 100ULL + (uint64_t)(((data[i] >> 4) & 0x0F) * 10 + (data[i] & 0x0F));
  }
  return digits * 10ULL;
}

void ft847EncodeFreqHz(uint64_t hz, uint8_t out[4]) {
  uint32_t units10 = (uint32_t)((hz / 10ULL) % 100000000ULL);
  for (int i = 3; i >= 0; --i) {
    const uint8_t pair = (uint8_t)(units10 % 100);
    units10 /= 100;
    out[i] = (uint8_t)(((pair / 10) << 4) | (pair % 10));
  }
}

bool ft847FreqFieldValid(const uint8_t data[4]) {
  for (int i = 0; i < 4; ++i) {
    if (((data[i] >> 4) & 0x0F) > 9 || (data[i] & 0x0F) > 9) return false;
  }
  const uint64_t hz = ft847DecodeFreqHz(data);
  return hz >= FT847_MIN_FREQ_HZ && hz <= FT847_MAX_FREQ_HZ;
}

void ft847BuildSetFreqFrame(uint64_t hz, uint8_t frame[5]) {
  ft847EncodeFreqHz(hz, frame);
  frame[4] = FT847_OP_SET_FREQ;
}

void ft847BuildSetModeFrame(uint8_t modeByte, uint8_t frame[5]) {
  ft847BuildFrame(FT847_OP_SET_MODE, frame);
  frame[0] = modeByte;
}

uint8_t ft847ModeBase(uint8_t modeByte) { return (uint8_t)(modeByte & ~FT847_MODE_NARROW_FLAG); }

bool ft847ModeNarrow(uint8_t modeByte) { return (modeByte & FT847_MODE_NARROW_FLAG) != 0; }

bool ft847ModeBaseKnown(uint8_t modeBase) {
  switch (modeBase) {
    case FT847_MODE_LSB:
    case FT847_MODE_USB:
    case FT847_MODE_CW:
    case FT847_MODE_CWR:
    case FT847_MODE_AM:
    case FT847_MODE_FM:
      return true;
    default:
      return false;
  }
}

bool ft847ModeCanBeNarrow(uint8_t modeBase) {
  return modeBase == FT847_MODE_CW || modeBase == FT847_MODE_CWR || modeBase == FT847_MODE_AM;
}

uint8_t ft847ModeByte(uint8_t modeBase, bool narrow) {
  return (uint8_t)(ft847ModeBase(modeBase) | (narrow ? FT847_MODE_NARROW_FLAG : 0));
}

uint8_t ft847SMeterDots(uint8_t rxStatus) { return (uint8_t)(rxStatus & 0x1F); }

void ft847DecodeSMeter(uint8_t rxStatus, uint8_t& sUnitsOut, uint8_t& dbOverS9Out) {
  const int dots = ft847SMeterDots(rxStatus);
  // dB relative to S9, as in Hamlib's ft847_get_smeter_level.
  int db;
  if (dots < 4) {
    db = -54 + dots * 2;
  } else if (dots < 20) {
    db = -48 + (dots - 3) * 3;
  } else {
    db = (dots - 19) * 5;
  }
  if (db <= 0) {
    // 6 dB per S unit, S0 at -54 dB.
    int s = (db + 54) / 6;
    if (s < 0) s = 0;
    if (s > 9) s = 9;
    sUnitsOut = (uint8_t)s;
    dbOverS9Out = 0;
  } else {
    sUnitsOut = 9;
    dbOverS9Out = (uint8_t)((db / 10) * 10);
  }
}

bool ft847SquelchOpen(uint8_t rxStatus) { return (rxStatus & 0x80) == 0; }

bool ft847TxStatusTransmitting(uint8_t txStatus) { return (txStatus & 0x80) == 0; }

uint8_t ft847TxMeter(uint8_t txStatus) { return (uint8_t)(txStatus & 0x1F); }
