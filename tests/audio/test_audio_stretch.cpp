// Tests for the speech speed time-stretch (firmware/audio_stretch.*).
#include "audio_stretch.h"
#include "test_runner.h"

#include <math.h>
#include <stdlib.h>

#include <vector>

namespace {

constexpr int kRate = 8000;

std::vector<int16_t> sine(float hz, float seconds, float amplitude = 10000.0f) {
  std::vector<int16_t> v((size_t)(kRate * seconds));
  for (size_t i = 0; i < v.size(); ++i) v[i] = (int16_t)lrintf(amplitude * sinf(6.28318530718f * hz * i / kRate));
  return v;
}

// Plays the whole clip through the stretch in blocks of the given size.
std::vector<int16_t> stretch(const std::vector<int16_t> &in, float speed, size_t block = 256) {
  AudioStretch s;
  stretchBegin(s, in.data(), in.size(), speed);
  std::vector<int16_t> out;
  std::vector<int16_t> buf(block);
  for (size_t n; (n = stretchNext(s, buf.data(), buf.size())) > 0;) out.insert(out.end(), buf.begin(), buf.begin() + n);
  return out;
}

// Upward zero crossings per second, skipping the first and last frame.
float crossingsPerSecond(const std::vector<int16_t> &v) {
  const size_t from = STRETCH_FRAME;
  const size_t to = v.size() - STRETCH_FRAME;
  int count = 0;
  for (size_t i = from + 1; i < to; ++i) {
    if (v[i - 1] < 0 && v[i] >= 0) ++count;
  }
  return count * (float)kRate / (float)(to - from);
}

int peak(const std::vector<int16_t> &v) {
  int p = 0;
  for (int16_t x : v) p = abs(x) > p ? abs(x) : p;
  return p;
}

// Speech-like test input: a 150 Hz voice whose loudness rises and falls.
std::vector<int16_t> voiced(float seconds) {
  std::vector<int16_t> v = sine(150.0f, seconds);
  for (size_t i = 0; i < v.size(); ++i) {
    const float env = 0.6f + 0.4f * sinf(6.28318530718f * 3.0f * i / kRate);
    v[i] = (int16_t)lrintf(v[i] * env);
  }
  return v;
}

}  // namespace

TEST(normal_speed_passes_the_clip_through) {
  const auto in = voiced(0.5f);
  const auto out = stretch(in, 1.0f);
  CHECK(out == in);
}

TEST(short_clip_passes_through) {
  const auto in = sine(440.0f, 0.03f);
  CHECK(stretch(in, 1.3f) == in);
}

TEST(block_size_does_not_change_the_output) {
  const auto in = voiced(0.6f);
  CHECK(stretch(in, 1.3f, 1) == stretch(in, 1.3f, 1000));
  CHECK(stretch(in, 0.8f, 7) == stretch(in, 0.8f, 256));
}

TEST(fast_is_shorter_by_the_speed) {
  const auto in = voiced(1.0f);
  const auto out = stretch(in, 1.3f);
  const long expected = lrintf(in.size() / 1.3f);
  CHECK(labs((long)out.size() - expected) <= STRETCH_FRAME);
}

TEST(slow_is_longer_by_the_speed) {
  const auto in = voiced(1.0f);
  const auto out = stretch(in, 0.8f);
  const long expected = lrintf(in.size() / 0.8f);
  CHECK(labs((long)out.size() - expected) <= STRETCH_FRAME);
}

TEST(pitch_stays_the_same) {
  for (float hz : {150.0f, 440.0f}) {
    const auto in = sine(hz, 1.0f);
    const float ref = crossingsPerSecond(in);
    for (float speed : {0.8f, 1.3f}) {
      const float got = crossingsPerSecond(stretch(in, speed));
      CHECK(fabsf(got - ref) <= ref * 0.02f);
    }
  }
}

TEST(level_does_not_grow) {
  const auto in = voiced(1.0f);
  for (float speed : {0.8f, 1.3f}) CHECK(peak(stretch(in, speed)) <= peak(in) * 105 / 100);
}

TEST(onset_and_end_are_kept_as_they_are) {
  const auto in = voiced(0.5f);
  const auto out = stretch(in, 1.3f);
  for (int i = 0; i < STRETCH_HOP; ++i) CHECK_EQ(out[i], in[i]);
  // The last samples are the clip's own ending.
  for (int i = 1; i <= STRETCH_HOP; ++i) CHECK_EQ(out[out.size() - i], in[in.size() - i]);
}
