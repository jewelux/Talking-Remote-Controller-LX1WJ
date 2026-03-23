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
    case 0x00: playClipProgmem(voice_lsb, voice_lsb_len); break;
    case 0x01: playClipProgmem(voice_usb, voice_usb_len); break;
    case 0x02: playClipProgmem(voice_am, voice_am_len); break;
    case 0x03: playClipProgmem(voice_cw, voice_cw_len); break;
    case 0x04: playClipProgmem(voice_rtty, voice_rtty_len); break;
    case 0x05: playClipProgmem(voice_fm, voice_fm_len); break;
    case 0x07: playClipProgmem(voice_cwr, voice_cwr_len); break;
    case 0x08: playClipProgmem(voice_rttyr, voice_rttyr_len); break;
    case 0x11: playClipProgmem(voice_digi, voice_digi_len); break;
    default: break;
  }
  playSilenceMs(200);
#else
  (void)mode;
#endif
}
