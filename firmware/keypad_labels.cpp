#include "keypad_labels.h"

#include <string.h>

namespace {

struct SpokenLabel {
  const char* label;  // trace label without '?'
  const char* words;  // clip tokens
};

// check_voice_tokens.py checks that every word here has a clip.
constexpr SpokenLabel kSpokenLabels[] = {
  {"A/B", "vfo a b"},
  {"A=B", "a equals b"},
  {"AGC", "agc"},
  {"ALC", "alc"},
  {"ANOTCH", "auto notch"},
  {"ANT", "antenna"},
  {"ATT", "attenuator"},
  {"BAND", "band"},
  {"BATTERY", "battery"},
  {"BAUD", "baud"},
  {"BK", "breakin"},
  {"BPF", "cw filterwidth"},
  {"BSTACK", "band stack"},
  {"BSTACK1", "band stack one"},
  {"BSTACK2", "band stack two"},
  {"BSTACK3", "band stack three"},
  {"CAL", "calibration"},
  {"CIVADDR", "c i"},
  {"CTCSS", "ctcss"},
  {"CWAUTO", "cw auto"},
  {"CWBFO", "cw b f o"},
  {"CWDELAY", "cw delay"},
  {"CWPITCH", "cw pitch"},
  {"CWSPEED", "cw speed"},
  {"CWWEIGHT", "cw weight"},
  {"DBF", "bandpassfilter"},
  {"DCS", "dcs"},
  {"EXPERIMENTAL", "experimental"},
  {"FILSHAPE", "filtershape"},
  {"FILWIDTH", "filterwidth"},
  {"FOCUS", "focus"},
  {"FREQ", "frequency"},
  {"HPF", "lowcut"},
  {"ID", "i d"},
  {"IF", "i f"},
  {"IPO", "preamplifier"},
  {"KEYREV", "paddle reverse"},
  {"KYR", "keyer"},
  {"LOCK", "lock"},
  {"LPF", "highcut"},
  {"MENU", "menu"},
  {"MICEQ", "tx equalizer"},
  {"MICGAIN", "micgain"},
  {"MODE", "mode"},
  {"MONITOR", "monitor"},
  {"MONLEVEL", "monitor level"},
  {"NAR", "narrow"},
  {"NB", "noiseblanker"},
  {"NBLEVEL", "noiseblanker level"},
  {"NOTCH", "notch filter"},
  {"NR", "noisereduction"},
  {"NRLEVEL", "noisereduction level"},
  {"PADDLE", "paddle"},
  {"PAVOLT", "p a volts"},
  {"PBT1", "pbt one"},
  {"PBT2", "pbt two"},
  {"PO", "tx power"},
  {"PREAMP", "preamplifier"},
  {"PROC", "processor"},
  {"PROCLEVEL", "processor level"},
  {"PROFILE", "profile"},
  {"PS", "power"},
  {"PS OFF", "power off"},
  {"PS ON", "power on"},
  {"PTT", "ptt"},
  {"PTT OFF", "ptt off"},
  {"PTT ON", "ptt on"},
  {"QSK", "q s k"},
  {"RANGE", "start stop"},
  {"RFPOWER", "power"},
  {"RIT", "rit"},
  {"RIT OFF", "rit off"},
  {"RITOFFSET", "rit"},
  {"ROUND", "round frequency"},
  {"ROW", "row"},
  {"RPT", "repeater"},
  {"RPT MINUS", "repeater minus"},
  {"RPT OFF", "repeater off"},
  {"RPT PLUS", "repeater plus"},
  {"RPTSHIFT", "repeater frequency"},
  {"RPTSHIFT1", "repeater frequency one"},
  {"RPTSHIFT2", "repeater frequency two"},
  {"RXTX", "transceiver"},
  {"SIDETONE", "sidetone"},
  {"SM", "s_meter"},
  {"SN", "s n"},
  {"SPEED", "speed"},
  {"SPLIT", "split"},
  {"SPLITTONE", "split tone"},
  {"START", "start"},
  {"STOP", "stop"},
  {"SWEEP", "sweep"},
  {"SWR", "swr"},
  {"SYNC A", "sync vfo a"},
  {"SYNC B", "sync vfo b"},
  {"TONE", "tone"},
  {"TONE CTCSS", "tone ctcss"},
  {"TONE DCS", "tone dcs"},
  {"TONE OFF", "tone off"},
  {"TONEDIR", "tone"},
  {"TRANSCEIVE", "transceive"},
  {"TUNE", "tune"},
  {"TUNER", "tuner"},
  {"TUNINGSPEECH", "auto frequency"},
  {"TXFREQ", "tx frequency"},
  {"TXHIGHCUT", "tx highcut"},
  {"TXLOWCUT", "tx lowcut"},
  {"USB", "usb"},
  {"VERBOSE", "verbose"},
  {"VERSION", "version"},
  {"VFO", "vfo"},
  {"VFO A", "vfo a"},
  {"VFO B", "vfo b"},
  {"VFOA", "vfo a"},
  {"VFOA FREQ", "vfo a frequency"},
  {"VFOA MODE", "vfo a mode"},
  {"VFOB", "vfo b"},
  {"VFOB FREQ", "vfo b frequency"},
  {"VFOB MODE", "vfo b mode"},
  {"VOLUME", "volume"},
  {"VOX", "vox"},
  {"VOXDELAY", "voxdelay"},
  {"VOXGAIN", "voxgain"},
};

}  // namespace

const char* spokenLabelWords(const char* label) {
  size_t len = strlen(label);
  if (len && label[len - 1] == '?') --len;
  for (const SpokenLabel& entry : kSpokenLabels) {
    if (strlen(entry.label) == len && strncmp(entry.label, label, len) == 0) return entry.words;
  }
  return nullptr;
}

namespace {

// The words a console reply says itself, so a command's label leaves them out.
bool repliedWord(const char* word, size_t len) {
  static const char* const kStates[] = {"ON", "OFF", "TOGGLE"};
  for (const char* state : kStates) {
    if (strlen(state) == len && strncmp(state, word, len) == 0) return true;
  }
  if (!len) return false;
  for (size_t i = 0; i < len; ++i) {
    if (word[i] < '0' || word[i] > '9') return false;
  }
  return true;
}

}  // namespace

void keypadCommandLabel(const char* command, char* out, size_t outSize) {
  if (!outSize) return;
  size_t len = 0;
  for (const char* c = command; *c && len + 1 < outSize; ++c) {
    if (*c != '?') out[len++] = *c;
  }
  while (len && out[len - 1] == ' ') --len;
  size_t word = len;
  while (word && out[word - 1] != ' ') --word;
  if (word && repliedWord(out + word, len - word)) {
    len = word;
    while (len && out[len - 1] == ' ') --len;
  }
  out[len] = '\0';
}
