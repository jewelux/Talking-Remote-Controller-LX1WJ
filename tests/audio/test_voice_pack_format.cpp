// Tests for the voice pack layout (firmware/voice_pack_format.*). The packs are built
// here the way voice_assets/build_voice_pack.py builds them.
#include "voice_pack_format.h"
#include "test_runner.h"

#include <string.h>

#include <string>
#include <vector>

namespace {

constexpr uint32_t kRate = 8000;

void putU16(std::vector<uint8_t> &b, size_t at, uint16_t v) {
  b[at] = (uint8_t)v;
  b[at + 1] = (uint8_t)(v >> 8);
}

void putU32(std::vector<uint8_t> &b, size_t at, uint32_t v) {
  for (int i = 0; i < 4; ++i) b[at + i] = (uint8_t)(v >> (8 * i));
}

// One clip per name, in the given order (the builder sorts; tests may not), each
// clip lengths[i] bytes long.
std::vector<uint8_t> makePack(const std::vector<std::string> &names, const std::vector<uint32_t> &lengths) {
  const size_t count = names.size();
  const uint32_t indexOffset = VOICE_PACK_HEADER_SIZE;
  const uint32_t dataOffset = (uint32_t)((indexOffset + count * VOICE_PACK_ENTRY_SIZE + 3) & ~(size_t)3);
  uint32_t total = dataOffset;
  std::vector<uint32_t> offsets;
  for (uint32_t len : lengths) {
    offsets.push_back(total);
    total = (total + len + 3) & ~3u;
  }
  std::vector<uint8_t> b(total, 0);
  memcpy(b.data(), "HTVP", 4);
  putU16(b, 4, VOICE_PACK_VERSION);
  putU16(b, 6, (uint16_t)count);
  putU32(b, 8, kRate);
  putU32(b, 12, indexOffset);
  putU32(b, 16, dataOffset);
  putU32(b, 20, total);
  putU32(b, 24, 0);
  for (size_t i = 0; i < count; ++i) {
    const size_t e = indexOffset + i * VOICE_PACK_ENTRY_SIZE;
    memcpy(&b[e], names[i].c_str(), names[i].size());
    putU32(b, e + VOICE_PACK_NAME_LEN, offsets[i]);
    putU32(b, e + VOICE_PACK_NAME_LEN + 4, lengths[i]);
  }
  return b;
}

std::vector<uint8_t> threeClips() { return makePack({"a", "noisereduction", "s_meter"}, {2, 6, 10}); }

VoicePackError parse(const std::vector<uint8_t> &b, VoicePackHeader &h, size_t capacity = 0x7F0000) {
  return voicePackParseHeader(b.data(), b.size(), kRate, capacity, &h);
}

}  // namespace

TEST(valid_pack_parses_and_checks) {
  const auto b = threeClips();
  VoicePackHeader h;
  CHECK_EQ(parse(b, h), VoicePackError::None);
  CHECK_EQ(h.count, 3);
  CHECK_EQ(h.totalSize, (uint32_t)b.size());
  CHECK_EQ(voicePackCheckIndex(b.data(), h), VoicePackError::None);
}

TEST(find_hits_every_clip_and_misses_others) {
  const auto b = threeClips();
  VoicePackHeader h;
  CHECK_EQ(parse(b, h), VoicePackError::None);
  CHECK_EQ(voicePackFind(b.data(), h, "a"), 0);
  CHECK_EQ(voicePackFind(b.data(), h, "noisereduction"), 1);
  CHECK_EQ(voicePackFind(b.data(), h, "s_meter"), 2);
  CHECK_EQ(voicePackFind(b.data(), h, "noise"), -1);
  CHECK_EQ(voicePackFind(b.data(), h, ""), -1);
  CHECK_EQ(voicePackFind(b.data(), h, "zzz"), -1);
}

TEST(entry_gives_name_offset_and_length) {
  const auto b = threeClips();
  VoicePackHeader h;
  CHECK_EQ(parse(b, h), VoicePackError::None);
  const VoicePackEntry e = voicePackEntryAt(b.data(), h, 2);
  CHECK_EQ(e.name, "s_meter");
  CHECK_EQ(e.length, 10u);
  CHECK_EQ(e.offset % 4, 0u);
  CHECK(e.offset >= h.dataOffset);
}

TEST(erased_flash_is_missing) {
  std::vector<uint8_t> b(64, 0xFF);
  VoicePackHeader h;
  CHECK_EQ(parse(b, h), VoicePackError::Magic);
}

TEST(short_buffer_is_missing) {
  auto b = threeClips();
  VoicePackHeader h;
  CHECK_EQ(voicePackParseHeader(b.data(), VOICE_PACK_HEADER_SIZE - 1, kRate, 0x7F0000, &h),
           VoicePackError::Magic);
}

TEST(other_version_is_rejected) {
  auto b = threeClips();
  putU16(b, 4, VOICE_PACK_VERSION + 1);
  VoicePackHeader h;
  CHECK_EQ(parse(b, h), VoicePackError::Version);
}

TEST(other_sample_rate_is_rejected) {
  auto b = threeClips();
  putU32(b, 8, 11025);
  VoicePackHeader h;
  CHECK_EQ(parse(b, h), VoicePackError::SampleRate);
}

TEST(pack_larger_than_partition_is_rejected) {
  const auto b = threeClips();
  VoicePackHeader h;
  CHECK_EQ(parse(b, h, b.size() - 1), VoicePackError::Layout);
}

TEST(index_running_into_data_is_rejected) {
  auto b = threeClips();
  putU16(b, 6, 1000);
  VoicePackHeader h;
  CHECK_EQ(parse(b, h), VoicePackError::Layout);
}

TEST(huge_offsets_do_not_wrap) {
  auto b = threeClips();
  putU32(b, 12, 0xFFFFFFF0u);
  VoicePackHeader h;
  CHECK_EQ(parse(b, h), VoicePackError::Layout);
}

TEST(empty_pack_is_rejected) {
  auto b = makePack({}, {});
  VoicePackHeader h;
  CHECK_EQ(parse(b, h), VoicePackError::Layout);
}

TEST(clip_past_the_end_is_rejected) {
  auto b = threeClips();
  VoicePackHeader h;
  CHECK_EQ(parse(b, h), VoicePackError::None);
  putU32(b, VOICE_PACK_HEADER_SIZE + 2 * VOICE_PACK_ENTRY_SIZE + VOICE_PACK_NAME_LEN + 4, 1000);
  CHECK_EQ(voicePackCheckIndex(b.data(), h), VoicePackError::Entry);
}

TEST(clip_in_the_index_area_is_rejected) {
  auto b = threeClips();
  VoicePackHeader h;
  CHECK_EQ(parse(b, h), VoicePackError::None);
  putU32(b, VOICE_PACK_HEADER_SIZE + VOICE_PACK_NAME_LEN, VOICE_PACK_HEADER_SIZE);
  CHECK_EQ(voicePackCheckIndex(b.data(), h), VoicePackError::Entry);
}

TEST(odd_length_clip_is_rejected) {
  auto b = makePack({"a"}, {3});
  VoicePackHeader h;
  CHECK_EQ(parse(b, h), VoicePackError::None);
  CHECK_EQ(voicePackCheckIndex(b.data(), h), VoicePackError::Entry);
}

TEST(name_without_terminator_is_rejected) {
  auto b = threeClips();
  VoicePackHeader h;
  CHECK_EQ(parse(b, h), VoicePackError::None);
  memset(&b[VOICE_PACK_HEADER_SIZE], 'x', VOICE_PACK_NAME_LEN);
  CHECK_EQ(voicePackCheckIndex(b.data(), h), VoicePackError::Entry);
}

TEST(unsorted_index_is_rejected) {
  auto b = makePack({"b", "a"}, {2, 2});
  VoicePackHeader h;
  CHECK_EQ(parse(b, h), VoicePackError::None);
  CHECK_EQ(voicePackCheckIndex(b.data(), h), VoicePackError::Order);
}

TEST(duplicate_name_is_rejected) {
  auto b = makePack({"a", "a"}, {2, 2});
  VoicePackHeader h;
  CHECK_EQ(parse(b, h), VoicePackError::None);
  CHECK_EQ(voicePackCheckIndex(b.data(), h), VoicePackError::Order);
}
