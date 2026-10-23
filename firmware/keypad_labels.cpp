#include "keypad_labels.h"

#include <string.h>

namespace {

struct SpokenLabel {
  const char* label;  // trace label without '?'
  const char* words;  // clip tokens
};

// check_voice_tokens.py checks that every word here has a clip.
constexpr SpokenLabel kSpokenLabels[] = {
  {"A=B", "a equals b"},
  {"AGC", "agc"},
  {"ANOTCH", "auto notch"},
  {"ANT", "antenna"},
  {"ATT", "attenuator"},
  {"BAUD", "baud"},
  {"BK", "b k"},
  {"BPF", "b p f"},
  {"BSTACK", "band stack"},
  {"CIVADDR", "c i"},
  {"CTCSS", "ctcss"},
  {"CWDELAY", "cw delay"},
  {"CWPITCH", "cw pitch"},
  {"CWSPEED", "cw speed"},
  {"CWWEIGHT", "cw weight"},
  {"DBF", "bandpassfilter"},
  {"DCS", "dcs"},
  {"FILSHAPE", "filtershape"},
  {"FILWIDTH", "filterwidth"},
  {"FREQ", "frequency"},
  {"HPF", "lowcut"},
  {"IPO", "preamplifier"},
  {"KEYREV", "key reverse"},
  {"KYR", "k y r"},
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
  {"NOTCH", "notch"},
  {"NR", "noisereduction"},
  {"NRLEVEL", "noisereduction level"},
  {"PADDLE", "paddle"},
  {"PBT1", "pbt one"},
  {"PBT2", "pbt two"},
  {"PREAMP", "preamplifier"},
  {"PROC", "processor"},
  {"PROCLEVEL", "processor level"},
  {"PROFILE", "profile"},
  {"QSK", "q s k"},
  {"RFPOWER", "power"},
  {"RIT", "rit"},
  {"RITOFFSET", "rit"},
  {"ROUND", "frequency"},
  {"ROW", "row"},
  {"RPT", "repeater"},
  {"RXTX", "transceiver"},
  {"SIDETONE", "sidetone"},
  {"SPEED", "speed"},
  {"SPLIT", "split"},
  {"SPLITTONE", "split tone"},
  {"TONE", "tone"},
  {"TONEDIR", "tone"},
  {"TRANSCEIVE", "transceive"},
  {"TUNE", "tune"},
  {"TUNER", "tuner"},
  {"TXFREQ", "tx frequency"},
  {"TXHIGHCUT", "tx highcut"},
  {"TXLOWCUT", "tx lowcut"},
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
