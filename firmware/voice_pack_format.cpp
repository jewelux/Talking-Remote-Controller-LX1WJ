#include "voice_pack_format.h"

#include <string.h>

namespace {

uint16_t readU16(const uint8_t* p) { return (uint16_t)(p[0] | (p[1] << 8)); }

uint32_t readU32(const uint8_t* p) {
  return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

const uint8_t* entryPtr(const uint8_t* pack, const VoicePackHeader& h, uint16_t i) {
  return pack + h.indexOffset + (size_t)i * VOICE_PACK_ENTRY_SIZE;
}

const char* entryName(const uint8_t* entry) { return (const char*)entry; }

}  // namespace

const char* voicePackErrorText(VoicePackError e) {
  switch (e) {
    case VoicePackError::None: return "ok";
    case VoicePackError::Magic: return "no voice pack";
    case VoicePackError::Version: return "unknown version";
    case VoicePackError::SampleRate: return "wrong sample rate";
    case VoicePackError::Layout: return "bad layout";
    case VoicePackError::Entry: return "bad clip entry";
    case VoicePackError::Order: return "index not sorted";
    case VoicePackError::Crc: return "checksum mismatch";
  }
  return "?";
}

VoicePackError voicePackParseHeader(const uint8_t* buf, size_t bufLen, uint32_t sampleRate,
                                    size_t capacity, VoicePackHeader* out) {
  if (bufLen < VOICE_PACK_HEADER_SIZE || memcmp(buf, "HTVP", 4) != 0) return VoicePackError::Magic;
  VoicePackHeader h;
  h.version = readU16(buf + 4);
  h.count = readU16(buf + 6);
  h.sampleRate = readU32(buf + 8);
  h.indexOffset = readU32(buf + 12);
  h.dataOffset = readU32(buf + 16);
  h.totalSize = readU32(buf + 20);
  h.dataCrc = readU32(buf + 24);
  if (h.version != VOICE_PACK_VERSION) return VoicePackError::Version;
  if (h.sampleRate != sampleRate) return VoicePackError::SampleRate;
  // 64-bit sums, so huge values cannot wrap past the checks.
  const uint64_t indexEnd = (uint64_t)h.indexOffset + (uint64_t)h.count * VOICE_PACK_ENTRY_SIZE;
  if (h.count == 0 || h.indexOffset < VOICE_PACK_HEADER_SIZE || indexEnd > h.dataOffset ||
      h.dataOffset > h.totalSize || h.totalSize > capacity) {
    return VoicePackError::Layout;
  }
  *out = h;
  return VoicePackError::None;
}

VoicePackError voicePackCheckIndex(const uint8_t* pack, const VoicePackHeader& h) {
  const char* prev = nullptr;
  for (uint16_t i = 0; i < h.count; ++i) {
    const uint8_t* e = entryPtr(pack, h, i);
    const char* name = entryName(e);
    if (name[0] == '\0' || memchr(name, '\0', VOICE_PACK_NAME_LEN) == nullptr) return VoicePackError::Entry;
    const uint32_t offset = readU32(e + VOICE_PACK_NAME_LEN);
    const uint32_t length = readU32(e + VOICE_PACK_NAME_LEN + 4);
    if (offset < h.dataOffset || (offset & 3) != 0 || length == 0 || (length & 1) != 0 ||
        (uint64_t)offset + length > h.totalSize) {
      return VoicePackError::Entry;
    }
    if (prev && strcmp(prev, name) >= 0) return VoicePackError::Order;
    prev = name;
  }
  return VoicePackError::None;
}

VoicePackEntry voicePackEntryAt(const uint8_t* pack, const VoicePackHeader& h, uint16_t i) {
  const uint8_t* e = entryPtr(pack, h, i);
  return {entryName(e), readU32(e + VOICE_PACK_NAME_LEN), readU32(e + VOICE_PACK_NAME_LEN + 4)};
}

int voicePackFind(const uint8_t* pack, const VoicePackHeader& h, const char* name) {
  int lo = 0;
  int hi = (int)h.count - 1;
  while (lo <= hi) {
    const int mid = (lo + hi) / 2;
    const int cmp = strcmp(name, entryName(entryPtr(pack, h, (uint16_t)mid)));
    if (cmp == 0) return mid;
    if (cmp < 0) hi = mid - 1;
    else lo = mid + 1;
  }
  return -1;
}
