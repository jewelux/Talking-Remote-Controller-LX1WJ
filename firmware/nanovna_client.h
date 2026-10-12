#pragma once

#include <Arduino.h>

#include "nanovna_codec.h"

// A NanoVNA on the radio link (PROTO_NANOVNA). It runs the device's shell commands without holding
// up the main loop: a job is a few commands in a row, pollNanoVna() moves it on, and the job's
// callback hears how it ended. The client never speaks; the keys and the console do.
//
// Every job starts with a bare CR and waits for the prompt, so whatever was left on the shell line
// goes first. Each command's reply counts from its echo to the prompt after it, so a late reply to
// a command that timed out is skipped; between jobs, what the device sends is dropped. Nothing is
// sent to the device unless a key or a console command asks: CAT polls once piled up on its shell
// line and hung it.

enum class NanoVnaStatus : uint8_t {
  Ok,
  NotAvailable,  // the active profile is not a NanoVNA
  NotConnected,  // its serial port is not open, or closed during the job
  Busy,          // another job runs
  BadRange,      // the start is not below the stop
  Timeout,       // no prompt within the step's time
  DeviceError,   // the device refused a command; NanoVnaResult::text has its reply
  NoData,        // the scan gave no point
};

struct NanoVnaResult {
  NanoVnaStatus status = NanoVnaStatus::Ok;
  // Measure: the range scanned and what it gave.
  uint32_t startHz = 0;
  uint32_t stopHz = 0;
  NanoVnaSweepSummary summary;
  // Battery: the vbat reply.
  uint32_t millivolts = 0;
  // Version and serial number: the reply's first line. DeviceError: the device's error line.
  char text[64] = "";
};

using NanoVnaDone = void (*)(const NanoVnaResult& result);

// No recall before a measurement: the calibration the device has loaded.
constexpr int8_t kNanoVnaCalAuto = -1;

// What the keys set, kept in NVS. Starts on 80 m with the device's own calibration.
struct NanoVnaSetup {
  uint32_t startHz;
  uint32_t stopHz;
  int8_t calSlot;  // 0..kNanoVnaCalSlots-1, or kNanoVnaCalAuto
};

void initNanoVna();
void pollNanoVna();
// From pollRadioLinkEvents(): the serial port went. A job still running ends NotConnected; the
// link monitor says "usb off", so its callback need not.
void nanoVnaLinkClosed();

bool nanoVnaBusy();
const NanoVnaSetup& nanoVnaSetup();
// Each keeps the setup in NVS. The frequencies are checked by the caller (nanoVnaFrequencyValid).
void nanoVnaSetStart(uint32_t hz);
void nanoVnaSetStop(uint32_t hz);
void nanoVnaSetRange(uint32_t startHz, uint32_t stopHz);
void nanoVnaSetCalSlot(int8_t slot);
// The last measurement that worked, or nullptr.
const NanoVnaResult* nanoVnaLastMeasure();

// Each returns Ok once the job runs, and its callback follows from pollNanoVna(). Any other
// status means it did not start, and done is not called.
//
// Measure: recall of the calibration slot (none for auto), the device's own sweep set to the range
// so its screen shows it, the scan, and resume, which also runs after a failed scan. BadRange when
// the start is not below the stop.
NanoVnaStatus nanoVnaStartMeasure(NanoVnaDone done);
NanoVnaStatus nanoVnaStartVersion(NanoVnaDone done);
// "SN": the serial number, in NanoVnaResult::text (the F V2 has it; older models refuse it).
NanoVnaStatus nanoVnaStartSerialNumber(NanoVnaDone done);
// "vbat": the battery voltage; NoData when the reply has no number.
NanoVnaStatus nanoVnaStartBattery(NanoVnaDone done);
// Recall of calibration slot (0..kNanoVnaCalSlots-1) alone.
NanoVnaStatus nanoVnaStartRecall(uint8_t slot, NanoVnaDone done);
// For the console: command as it is, each reply line printed to out.
NanoVnaStatus nanoVnaStartRaw(const char* command, Print& out, NanoVnaDone done);

const char* nanoVnaStatusName(NanoVnaStatus status);
// VNA? on the console: the setup, the link, the job and the last measurement.
void nanoVnaPrintStatus(Print& out);
