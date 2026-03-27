#include "radio_mode.h"

#include "radio_globals.h"
#include "ui_speech.h"

const char* modeToString(uint8_t mode) {
  switch (mode) {
    case 0x00: return "LSB";
    case 0x01: return "USB";
    case 0x02: return "AM";
    case 0x03: return "CW";
    case 0x04: return "RTTY";
    case 0x05: return "FM";
    case 0x06: return "WFM";
    case 0x07: return "CWR";
    case 0x08: return "RTTY-R";
    case 0x11: return "DIGI";
    default: return "UNK";
  }
}

void speakMode(uint8_t mode) {
#if HAVE_MODE_VOICE
#if defined(HAS_VOICE_voice_mode)
  if (!g_suppressModePrefixOnce) {
    playClipProgmem(voice_mode, voice_mode_len);
    playSilenceMs(120);
  } else {
    g_suppressModePrefixOnce = false;
  }
#endif
  switch (mode) {
    case 0x00: speakToken("lsb"); break;
    case 0x01: speakToken("usb"); break;
    case 0x02: speakToken("am"); break;
    case 0x03: speakToken("cw"); break;
    case 0x04: speakToken("rtty"); break;
    case 0x05: speakToken("fm"); break;
    case 0x06: speakToken("wfm"); break;
    case 0x07: speakToken("cwr"); break;
    case 0x08: speakToken("rttyr"); break;
    case 0x11: speakToken("digi"); break;
    default: break;
  }
  playSilenceMs(200);
#else
  (void)mode;
#endif
}
