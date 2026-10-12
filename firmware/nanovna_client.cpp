#include "nanovna_client.h"

#include "radio_catalog.h"
#include "radio_prefs.h"
#include "transport_serial.h"
#include "transport_usb_host.h"

namespace {

enum class Step : uint8_t { Sync, Recall, Sweep, Scan, Resume, Version, SerialNumber, Battery, Raw };

constexpr size_t kMaxSteps = 6;
constexpr size_t kMaxCommand = 64;
constexpr uint32_t kSyncTimeoutMs = 1000;
constexpr uint32_t kCommandTimeoutMs = 2000;
// The scan replies only once the sweep is done; 101 points take a few seconds at most.
constexpr uint32_t kScanTimeoutMs = 20000;
// Bytes taken per pollNanoVna(), so a scan reply does not hold up the keypad.
constexpr size_t kPollBudgetBytes = 512;

constexpr NanoVnaSetup kDefaultSetup = {3500000, 3800000, kNanoVnaCalAuto};

struct Job {
  bool active = false;
  Step steps[kMaxSteps];
  uint8_t count = 0;
  uint8_t index = 0;
  char command[kMaxCommand] = "";  // the line of the running step, without the CR
  uint32_t sentMs = 0;
  bool echoSeen = false;
  bool stepFailed = false;
  NanoVnaStatus failure = NanoVnaStatus::Ok;  // the first failure; the job may still run resume
  bool measure = false;
  uint8_t recallSlot = 0;
  char rawCommand[kMaxCommand] = "";
  Print* rawOut = nullptr;
  NanoVnaDone done = nullptr;
  NanoVnaResult result;
};

Job s_job;
NanoVnaReplyReader s_reader;
NanoVnaSetup s_setup = kDefaultSetup;
NanoVnaResult s_lastMeasure;
bool s_hasLastMeasure = false;

bool nanoVnaActive() {
  return currentProtocolType() == PROTO_NANOVNA;
}

bool linkReady() {
  return currentConnectionProfile().port != RadioPort::UsbHost || usbHostSerialReady();
}

uint32_t timeoutFor(Step step) {
  switch (step) {
    case Step::Sync: return kSyncTimeoutMs;
    case Step::Scan: return kScanTimeoutMs;
    default: return kCommandTimeoutMs;
  }
}

void saveSetup() {
  saveNanoVnaSetupToNvs(s_setup.startHz, s_setup.stopHz, s_setup.calSlot);
}

// The line of step, without the CR. Sync sends only the CR.
void buildCommand(Step step, char* out, size_t outLen) {
  out[0] = '\0';
  switch (step) {
    case Step::Sync: break;
    case Step::Recall: (void)nanoVnaRecallCommand(s_job.recallSlot, out, outLen); break;
    case Step::Sweep: (void)nanoVnaSweepCommand(s_job.result.startHz, s_job.result.stopHz, out, outLen); break;
    case Step::Scan: (void)nanoVnaScanCommand(s_job.result.startHz, s_job.result.stopHz, out, outLen); break;
    case Step::Resume: strlcpy(out, "resume", outLen); break;
    case Step::Version: strlcpy(out, "version", outLen); break;
    case Step::SerialNumber: strlcpy(out, "SN", outLen); break;
    case Step::Battery: strlcpy(out, "vbat", outLen); break;
    case Step::Raw: strlcpy(out, s_job.rawCommand, outLen); break;
  }
}

void finish(NanoVnaStatus status) {
  s_job.active = false;
  s_job.result.status = status;
  const bool measured = status == NanoVnaStatus::Ok && s_job.measure;
  if (measured) {
    s_lastMeasure = s_job.result;
    s_hasLastMeasure = true;
  }
  if (status == NanoVnaStatus::Ok) {
    if (measured) {
      const NanoVnaSweepSummary& s = s_job.result.summary;
      Serial0.printf("[VNA] %u points, best SWR %.2f at %lu Hz, worst SWR %.2f at %lu Hz\n", (unsigned)s.points,
                     (double)s.bestSwr, (unsigned long)s.bestHz, (double)s.worstSwr, (unsigned long)s.worstHz);
    }
  } else {
    Serial0.printf("[VNA] %s%s%s\n", nanoVnaStatusName(status), s_job.result.text[0] ? ": " : "", s_job.result.text);
  }
  const NanoVnaDone done = s_job.done;
  s_job.done = nullptr;
  s_job.rawOut = nullptr;
  if (done) done(s_job.result);
}

bool writeLine(const char* line) {
  const size_t len = strlen(line);
  return (len == 0 || serialTransportWrite(reinterpret_cast<const uint8_t*>(line), len) == len) &&
         serialTransportWrite(reinterpret_cast<const uint8_t*>("\r"), 1) == 1;
}

// False when the line could not be written.
bool sendStep() {
  const Step step = s_job.steps[s_job.index];
  buildCommand(step, s_job.command, sizeof(s_job.command));
  s_job.echoSeen = false;
  s_job.stepFailed = false;
  s_reader.reset();
  if (step != Step::Sync) Serial0.printf("[VNA] > %s\n", s_job.command);
  s_job.sentMs = millis();
  return writeLine(s_job.command);
}

// A write failed mid-job: the port went, or the device stopped reading.
NanoVnaStatus writeFailure() {
  return linkReady() ? NanoVnaStatus::Timeout : NanoVnaStatus::NotConnected;
}

// The step got no prompt in time. A scan pauses the device's sweep, so a measurement still sends
// resume, without waiting for it: the device takes it once it is done, and its reply is dropped
// with the rest of the late output.
void timedOut() {
  for (uint8_t i = s_job.index + 1; i < s_job.count; ++i) {
    if (s_job.steps[i] != Step::Resume) continue;
    Serial0.println("[VNA] > resume (not waited for)");
    (void)writeLine("resume");
    break;
  }
  finish(NanoVnaStatus::Timeout);
}

// The step ended at the prompt: the next one, the resume after a failure, or the end.
void stepDone() {
  const Step step = s_job.steps[s_job.index];
  if (s_job.failure == NanoVnaStatus::Ok) {
    if (s_job.stepFailed && step == Step::Resume) {
      // The scan is in: a device that refuses resume keeps the measurement.
      Serial0.printf("[VNA] resume refused: %s\n", s_job.result.text);
      s_job.result.text[0] = '\0';
    } else if (s_job.stepFailed) {
      s_job.failure = NanoVnaStatus::DeviceError;
    } else if (step == Step::Scan && s_job.result.summary.points == 0) {
      s_job.failure = NanoVnaStatus::NoData;
    } else if (step == Step::Battery && s_job.result.millivolts == 0) {
      s_job.failure = NanoVnaStatus::NoData;
    }
  }
  uint8_t next = s_job.index + 1;
  // After a failure only the resume runs, so the device sweeps again.
  if (s_job.failure != NanoVnaStatus::Ok) {
    while (next < s_job.count && s_job.steps[next] != Step::Resume) ++next;
  }
  if (next >= s_job.count) {
    finish(s_job.failure);
    return;
  }
  s_job.index = next;
  if (!sendStep()) finish(writeFailure());
}

// A step's reply starts after its echo. Lines before it are late output of an earlier command, one
// that timed out, and are skipped.
void handleLine(const char* line) {
  const Step step = s_job.steps[s_job.index];
  if (!s_job.echoSeen) {
    s_job.echoSeen = nanoVnaIsEcho(line, s_job.command);
    return;
  }
  if (step == Step::Sync) return;
  if (step == Step::Raw) {
    if (s_job.rawOut) s_job.rawOut->println(line);
    return;
  }
  if (nanoVnaReplyIsError(line, s_job.command)) {
    if (!s_job.stepFailed && s_job.failure == NanoVnaStatus::Ok) {
      strlcpy(s_job.result.text, line, sizeof(s_job.result.text));
    }
    s_job.stepFailed = true;
    return;
  }
  switch (step) {
    case Step::Scan: {
      NanoVnaPoint p;
      if (nanoVnaParseScanLine(line, p)) {
        s_job.result.summary.add(p.hz, nanoVnaSwr(p.re, p.im));
      } else if (line[0]) {
        Serial0.printf("[VNA] < %s\n", line);
      }
      return;
    }
    case Step::Battery:
      if (line[0]) Serial0.printf("[VNA] < %s\n", line);
      if (!s_job.result.millivolts) (void)nanoVnaParseMillivolts(line, s_job.result.millivolts);
      return;
    case Step::Version:
    case Step::SerialNumber:
      if (line[0] && !s_job.result.text[0]) strlcpy(s_job.result.text, line, sizeof(s_job.result.text));
      if (line[0]) Serial0.printf("[VNA] < %s\n", line);
      return;
    default:
      if (line[0]) Serial0.printf("[VNA] < %s\n", line);
      return;
  }
}

// What a job needs besides its steps.
struct JobParams {
  bool measure = false;
  uint8_t recallSlot = 0;
  const char* rawCommand = "";
  Print* rawOut = nullptr;
};

NanoVnaStatus startJob(const Step* steps, uint8_t count, NanoVnaDone done, const JobParams& params = {}) {
  if (!nanoVnaActive()) return NanoVnaStatus::NotAvailable;
  if (s_job.active) return NanoVnaStatus::Busy;
  if (!linkReady()) return NanoVnaStatus::NotConnected;
  s_job.active = true;
  s_job.measure = params.measure;
  s_job.recallSlot = params.recallSlot;
  strlcpy(s_job.rawCommand, params.rawCommand, sizeof(s_job.rawCommand));
  s_job.rawOut = params.rawOut;
  s_job.count = 0;
  s_job.steps[s_job.count++] = Step::Sync;
  for (uint8_t i = 0; i < count && s_job.count < kMaxSteps; ++i) s_job.steps[s_job.count++] = steps[i];
  s_job.index = 0;
  s_job.failure = NanoVnaStatus::Ok;
  s_job.done = done;
  s_job.result = NanoVnaResult{};
  s_job.result.startHz = s_setup.startHz;
  s_job.result.stopHz = s_setup.stopHz;
  serialTransportFlushInput();
  if (!sendStep()) {
    // Not started, so done is not called.
    s_job.active = false;
    s_job.done = nullptr;
    s_job.rawOut = nullptr;
    Serial0.println("[VNA] not connected: the command could not be written");
    return NanoVnaStatus::NotConnected;
  }
  return NanoVnaStatus::Ok;
}

}  // namespace

void initNanoVna() {
  NanoVnaSetup saved = kDefaultSetup;
  if (loadNanoVnaSetupFromNvs(saved.startHz, saved.stopHz, saved.calSlot) &&
      nanoVnaFrequencyValid(saved.startHz) && nanoVnaFrequencyValid(saved.stopHz) &&
      saved.calSlot >= kNanoVnaCalAuto && saved.calSlot < (int8_t)kNanoVnaCalSlots) {
    s_setup = saved;
  }
}

void pollNanoVna() {
  if (!s_job.active) {
    // Between jobs, what the device still sends (the rest of a reply that timed out) is dropped,
    // so the USB read never waits on a full buffer and the device is not held up.
    if (nanoVnaActive()) {
      for (size_t n = 0; n < kPollBudgetBytes && serialTransportRead() >= 0; ++n) {
      }
    }
    return;
  }
  // Another profile was chosen: the job is dropped without a word.
  if (!nanoVnaActive()) {
    s_job.active = false;
    s_job.done = nullptr;
    s_job.rawOut = nullptr;
    return;
  }
  for (size_t n = 0; n < kPollBudgetBytes && serialTransportAvailable(); ++n) {
    const int c = serialTransportRead();
    if (c < 0) break;
    switch (s_reader.feed((char)c)) {
      case NanoVnaReplyReader::Event::Line:
        handleLine(s_reader.line());
        break;
      case NanoVnaReplyReader::Event::Prompt:
        // A prompt before the echo ends an earlier command. The sync takes any prompt: the CR's
        // own follows, and the next step skips it.
        if (!s_job.echoSeen && s_job.steps[s_job.index] != Step::Sync) break;
        stepDone();
        if (!s_job.active) return;
        break;
      case NanoVnaReplyReader::Event::None:
        break;
    }
  }
  // Bytes still waiting may hold the prompt: the next pass reads them first.
  if (serialTransportAvailable()) return;
  if (millis() - s_job.sentMs > timeoutFor(s_job.steps[s_job.index])) timedOut();
}

void nanoVnaLinkClosed() {
  if (s_job.active) finish(NanoVnaStatus::NotConnected);
}

bool nanoVnaBusy() { return s_job.active; }

const NanoVnaSetup& nanoVnaSetup() { return s_setup; }

void nanoVnaSetStart(uint32_t hz) {
  s_setup.startHz = hz;
  saveSetup();
}

void nanoVnaSetStop(uint32_t hz) {
  s_setup.stopHz = hz;
  saveSetup();
}

void nanoVnaSetRange(uint32_t startHz, uint32_t stopHz) {
  s_setup.startHz = startHz;
  s_setup.stopHz = stopHz;
  saveSetup();
}

void nanoVnaSetCalSlot(int8_t slot) {
  s_setup.calSlot = slot;
  saveSetup();
}

const NanoVnaResult* nanoVnaLastMeasure() {
  return s_hasLastMeasure ? &s_lastMeasure : nullptr;
}

NanoVnaStatus nanoVnaStartMeasure(NanoVnaDone done) {
  // The device would set the stop to the start and leave its screen on zero span.
  if (nanoVnaActive() && !nanoVnaRangeValid(s_setup.startHz, s_setup.stopHz)) return NanoVnaStatus::BadRange;
  Step steps[4];
  uint8_t count = 0;
  if (s_setup.calSlot != kNanoVnaCalAuto) steps[count++] = Step::Recall;
  steps[count++] = Step::Sweep;
  steps[count++] = Step::Scan;
  steps[count++] = Step::Resume;
  JobParams params;
  params.measure = true;
  params.recallSlot = s_setup.calSlot < 0 ? 0 : (uint8_t)s_setup.calSlot;
  return startJob(steps, count, done, params);
}

NanoVnaStatus nanoVnaStartVersion(NanoVnaDone done) {
  const Step steps[] = {Step::Version};
  return startJob(steps, 1, done);
}

NanoVnaStatus nanoVnaStartSerialNumber(NanoVnaDone done) {
  const Step steps[] = {Step::SerialNumber};
  return startJob(steps, 1, done);
}

NanoVnaStatus nanoVnaStartBattery(NanoVnaDone done) {
  const Step steps[] = {Step::Battery};
  return startJob(steps, 1, done);
}

NanoVnaStatus nanoVnaStartRecall(uint8_t slot, NanoVnaDone done) {
  const Step steps[] = {Step::Recall};
  JobParams params;
  params.recallSlot = slot;
  return startJob(steps, 1, done, params);
}

NanoVnaStatus nanoVnaStartRaw(const char* command, Print& out, NanoVnaDone done) {
  const Step steps[] = {Step::Raw};
  JobParams params;
  params.rawCommand = command;
  params.rawOut = &out;
  return startJob(steps, 1, done, params);
}

const char* nanoVnaStatusName(NanoVnaStatus status) {
  switch (status) {
    case NanoVnaStatus::Ok: return "ok";
    case NanoVnaStatus::NotAvailable: return "not a NanoVNA profile";
    case NanoVnaStatus::NotConnected: return "not connected";
    case NanoVnaStatus::Busy: return "busy";
    case NanoVnaStatus::BadRange: return "start not below stop";
    case NanoVnaStatus::Timeout: return "timeout";
    case NanoVnaStatus::DeviceError: return "device error";
    case NanoVnaStatus::NoData: return "no data";
  }
  return "?";
}

void nanoVnaPrintStatus(Print& out) {
  if (!nanoVnaActive()) {
    out.println("NanoVNA: profile 30 is not active");
  } else {
    out.printf("NanoVNA: %s, %s\n", linkReady() ? "connected" : "not connected", s_job.active ? "busy" : "idle");
  }
  out.printf("  range: %lu - %lu Hz", (unsigned long)s_setup.startHz, (unsigned long)s_setup.stopHz);
  const int band = nanoVnaBandMatching(s_setup.startHz, s_setup.stopHz);
  if (band >= 0) {
    const NanoVnaBand& b = nanoVnaBandAt((size_t)band);
    out.printf(" (%u %s)", (unsigned)b.wavelength, b.unit == NanoVnaBandUnit::Meters ? "m" : "cm");
  }
  out.println();
  if (s_setup.calSlot == kNanoVnaCalAuto) {
    out.println("  calibration: auto (as loaded on the device)");
  } else {
    out.printf("  calibration: slot %d (recalled before each measurement)\n", (int)s_setup.calSlot);
  }
  if (!s_hasLastMeasure) {
    out.println("  last measurement: none");
    return;
  }
  const NanoVnaSweepSummary& s = s_lastMeasure.summary;
  out.printf("  last measurement: %lu - %lu Hz, %u points, best SWR %.2f at %lu Hz, worst SWR %.2f at %lu Hz\n",
             (unsigned long)s_lastMeasure.startHz, (unsigned long)s_lastMeasure.stopHz, (unsigned)s.points,
             (double)s.bestSwr, (unsigned long)s.bestHz, (double)s.worstSwr, (unsigned long)s.worstHz);
}
