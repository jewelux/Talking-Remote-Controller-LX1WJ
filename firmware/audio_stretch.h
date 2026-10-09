// Speech speed: plays a PCM16 clip faster or slower without changing its pitch
// (WSOLA, waveform-similarity overlap-add). Arduino-free so the host tests can
// check it.
//
// The output is built from 20 ms frames, Hann-windowed and overlapped by half.
// Each next frame is taken around where the speed says the input should be,
// shifted by up to 5 ms to where it best continues the previous one, so the
// pitch periods line up and the overlap does not smear them. The clip's first
// half frame and its end are copied as they are, so word onsets and endings
// stay intact.
#pragma once

#include <stddef.h>
#include <stdint.h>

static constexpr int STRETCH_FRAME = 160;  // 20 ms at 8 kHz
static constexpr int STRETCH_HOP = STRETCH_FRAME / 2;
static constexpr int STRETCH_SEEK = 40;    // +-5 ms at 8 kHz

struct AudioStretch {
  const int16_t* in;
  size_t inLen;
  uint32_t speedQ16;  // input samples per output sample, 16.16 fixed point
  uint32_t frame;     // index of the next frame
  size_t prevPos;     // input position of the last frame
  float tail[STRETCH_HOP];  // the last frame's second half, windowed
  int16_t hop[STRETCH_HOP];
  size_t hopLen;
  size_t hopPos;
  size_t rawPos;  // input still to copy as it is, [rawPos, rawEnd)
  size_t rawEnd;
  bool framesDone;
};

// speed > 1 is faster, < 1 slower; 1 (or a clip too short to stretch) passes the
// samples through unchanged. in must stay valid until the clip is played.
void stretchBegin(AudioStretch& s, const int16_t* in, size_t inLen, float speed);
// Up to max output samples; 0 once the clip is done.
size_t stretchNext(AudioStretch& s, int16_t* out, size_t max);
