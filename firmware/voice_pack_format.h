// Layout of the voice pack (voices.bin), built by voice_assets/build_voice_pack.py and
// flashed into the "voices" partition. Arduino-free so the host tests can check it.
//
//   header (32 B)  magic "HTVP", u16 version, u16 count, u32 sample rate,
//                  u32 index offset, u32 data offset, u32 total size,
//                  u32 CRC32 of the data, u32 reserved
//   index          count x (char name[20], u32 offset, u32 length), sorted by name
//   data           PCM16 mono clips, each starting 4-byte aligned
//
// All numbers are little-endian; offsets count from the start of the pack.
#pragma once

#include <stddef.h>
#include <stdint.h>

static constexpr uint16_t VOICE_PACK_VERSION = 1;
static constexpr size_t VOICE_PACK_HEADER_SIZE = 32;
static constexpr size_t VOICE_PACK_NAME_LEN = 20;
static constexpr size_t VOICE_PACK_ENTRY_SIZE = VOICE_PACK_NAME_LEN + 8;

struct VoicePackHeader {
  uint16_t version;
  uint16_t count;
  uint32_t sampleRate;
  uint32_t indexOffset;
  uint32_t dataOffset;
  uint32_t totalSize;
  uint32_t dataCrc;
};

struct VoicePackEntry {
  const char* name;  // NUL-terminated inside the pack
  uint32_t offset;
  uint32_t length;  // bytes
};

enum class VoicePackError : uint8_t {
  None,
  Magic,
  Version,
  SampleRate,
  Layout,  // index or data outside the pack, or the pack larger than its partition
  Entry,   // a clip with a bad name, offset or length
  Order,   // the index is not sorted by name
  Crc,
};

const char* voicePackErrorText(VoicePackError e);

// Reads the header from the first VOICE_PACK_HEADER_SIZE bytes of buf and checks its
// magic, version and sample rate, and that the index and data fit in capacity bytes.
VoicePackError voicePackParseHeader(const uint8_t* buf, size_t bufLen, uint32_t sampleRate,
                                    size_t capacity, VoicePackHeader* out);

// Checks every index entry of the whole pack: a non-empty NUL-terminated name, names in
// strictly increasing order, and PCM16 data that is aligned and lies in the data area.
VoicePackError voicePackCheckIndex(const uint8_t* pack, const VoicePackHeader& h);

// Only for a pack that passed voicePackCheckIndex.
VoicePackEntry voicePackEntryAt(const uint8_t* pack, const VoicePackHeader& h, uint16_t i);
// The index of the clip called name, or -1.
int voicePackFind(const uint8_t* pack, const VoicePackHeader& h, const char* name);
