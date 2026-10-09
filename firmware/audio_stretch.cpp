#include "audio_stretch.h"

#include <math.h>
#include <string.h>

namespace {

// Periodic Hann window: w[i] + w[i + STRETCH_HOP] == 1, so two frames overlapped
// by half add up to the original level.
const float* hannWindow() {
  static float w[STRETCH_FRAME];
  static bool ready = false;
  if (!ready) {
    const float twoPi = 6.28318530718f;
    for (int i = 0; i < STRETCH_FRAME; ++i) w[i] = 0.5f - 0.5f * cosf(twoPi * i / STRETCH_FRAME);
    ready = true;
  }
  return w;
}

int16_t toSample(float v) {
  const long r = lrintf(v);
  if (r > 32767) return 32767;
  if (r < -32768) return -32768;
  return (int16_t)r;
}

// The input position in [lo, hi] whose first half frame best matches ref:
// highest correlation, normalized by the candidate's energy so loud stretches
// do not win just for being loud.
size_t bestMatch(const int16_t* x, const int16_t* ref, size_t lo, size_t hi) {
  int64_t energy = 0;
  for (int j = 0; j < STRETCH_HOP; ++j) energy += (int32_t)x[lo + j] * x[lo + j];
  size_t best = lo;
  float bestScore = -INFINITY;
  for (size_t p = lo; p <= hi; ++p) {
    if (p > lo) {
      energy -= (int32_t)x[p - 1] * x[p - 1];
      energy += (int32_t)x[p + STRETCH_HOP - 1] * x[p + STRETCH_HOP - 1];
    }
    int64_t corr = 0;
    for (int j = 0; j < STRETCH_HOP; ++j) corr += (int32_t)ref[j] * x[p + j];
    const float score = (float)corr / sqrtf((float)energy + 1.0f);
    if (score > bestScore) {
      bestScore = score;
      best = p;
    }
  }
  return best;
}

void finishRaw(AudioStretch& s, size_t from) {
  s.rawPos = from;
  s.rawEnd = s.inLen;
  s.framesDone = true;
}

// Fills s.hop with the next STRETCH_HOP output samples, or hands the rest of
// the clip to the raw copy.
void makeHop(AudioStretch& s) {
  const float* w = hannWindow();
  const int16_t* x = s.in;
  s.hopPos = 0;
  s.hopLen = 0;

  if (s.frame == 0) {
    // The first half frame as it is, so the word's onset is not faded in.
    memcpy(s.hop, x, sizeof(s.hop));
    for (int j = 0; j < STRETCH_HOP; ++j) s.tail[j] = x[STRETCH_HOP + j] * w[STRETCH_HOP + j];
    s.hopLen = STRETCH_HOP;
    s.prevPos = 0;
    s.frame = 1;
    return;
  }

  const size_t nominal = (size_t)(((uint64_t)s.frame * STRETCH_HOP * s.speedQ16) >> 16);
  const size_t last = s.inLen - STRETCH_FRAME;  // last position a whole frame fits
  const size_t lo = nominal > STRETCH_SEEK ? nominal - STRETCH_SEEK : 0;
  const size_t hi = nominal + STRETCH_SEEK < last ? nominal + STRETCH_SEEK : last;
  if (lo > hi) {
    // No whole frame left. The previous frame's tail plus the input that
    // follows it, windowed the other way, is that input itself.
    finishRaw(s, s.prevPos + STRETCH_HOP);
    return;
  }

  // The previous frame's natural continuation is what the new one should match.
  const size_t p = bestMatch(x, x + s.prevPos + STRETCH_HOP, lo, hi);
  for (int j = 0; j < STRETCH_HOP; ++j) {
    s.hop[j] = toSample(s.tail[j] + x[p + j] * w[j]);
    s.tail[j] = x[p + STRETCH_HOP + j] * w[STRETCH_HOP + j];
  }
  s.hopLen = STRETCH_HOP;
  s.prevPos = p;
  s.frame++;
}

}  // namespace

void stretchBegin(AudioStretch& s, const int16_t* in, size_t inLen, float speed) {
  memset(&s, 0, sizeof(s));
  s.in = in;
  s.inLen = inLen;
  s.speedQ16 = (uint32_t)lrintf(speed * 65536.0f);
  if (s.speedQ16 == 65536 || speed <= 0.0f || inLen < 2 * STRETCH_FRAME + STRETCH_SEEK) finishRaw(s, 0);
}

size_t stretchNext(AudioStretch& s, int16_t* out, size_t max) {
  size_t n = 0;
  while (n < max) {
    if (s.hopPos < s.hopLen) {
      size_t k = s.hopLen - s.hopPos;
      if (k > max - n) k = max - n;
      memcpy(out + n, s.hop + s.hopPos, k * sizeof(int16_t));
      s.hopPos += k;
      n += k;
    } else if (s.rawPos < s.rawEnd) {
      size_t k = s.rawEnd - s.rawPos;
      if (k > max - n) k = max - n;
      memcpy(out + n, s.in + s.rawPos, k * sizeof(int16_t));
      s.rawPos += k;
      n += k;
    } else if (s.framesDone) {
      break;
    } else {
      makeHop(s);
    }
  }
  return n;
}
