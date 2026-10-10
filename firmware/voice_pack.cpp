#include "voice_pack.h"

#include <Arduino.h>

#include "config_pins.h"
#include "esp_partition.h"
#include "esp_rom_crc.h"
#include "voice_pack_format.h"

// Data subtype of the "voices" partition in partitions.csv.
static constexpr esp_partition_subtype_t VOICE_PACK_SUBTYPE = (esp_partition_subtype_t)0x40;

static const uint8_t* s_pack = nullptr;
static VoicePackHeader s_header = {};
static char s_status[48] = "MISSING";

static bool fail(VoicePackError e) {
  if (e == VoicePackError::Magic) snprintf(s_status, sizeof(s_status), "MISSING");
  else snprintf(s_status, sizeof(s_status), "INVALID (%s)", voicePackErrorText(e));
  return false;
}

static bool mapPack() {
  const esp_partition_t* part = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, VOICE_PACK_SUBTYPE, "voices");
  if (!part) return fail(VoicePackError::Magic);

  uint8_t raw[VOICE_PACK_HEADER_SIZE];
  if (esp_partition_read(part, 0, raw, sizeof(raw)) != ESP_OK) return fail(VoicePackError::Magic);
  VoicePackHeader h;
  VoicePackError e = voicePackParseHeader(raw, sizeof(raw), I2S_SAMPLE_RATE, part->size, &h);
  if (e != VoicePackError::None) return fail(e);

  const void* mapped = nullptr;
  esp_partition_mmap_handle_t handle;
  if (esp_partition_mmap(part, 0, h.totalSize, ESP_PARTITION_MMAP_DATA, &mapped, &handle) != ESP_OK) {
    snprintf(s_status, sizeof(s_status), "INVALID (cannot map)");
    return false;
  }
  const uint8_t* pack = (const uint8_t*)mapped;
  if (esp_rom_crc32_le(0, pack + h.dataOffset, h.totalSize - h.dataOffset) != h.dataCrc) e = VoicePackError::Crc;
  else e = voicePackCheckIndex(pack, h);
  if (e != VoicePackError::None) {
    esp_partition_munmap(handle);
    return fail(e);
  }

  s_pack = pack;
  s_header = h;
  snprintf(s_status, sizeof(s_status), "OK");
  return true;
}

bool voicePackInit() {
  const bool ok = mapPack();
  Serial.print("VOICE PACK ");
  voicePackPrintStatus();
  return ok;
}

bool voicePackReady() { return s_pack != nullptr; }

void voicePackPrintStatus() {
  Serial.print(s_status);
  if (s_pack) {
    Serial.print(" (");
    Serial.print((int)s_header.count);
    Serial.print(" clips, ");
    Serial.print((unsigned long)s_header.totalSize);
    Serial.print(" bytes)");
  }
  Serial.println();
}

bool voicePackClipAt(size_t i, VoiceClip* out) {
  if (!s_pack || i >= s_header.count) return false;
  const VoicePackEntry e = voicePackEntryAt(s_pack, s_header, (uint16_t)i);
  *out = {e.name, s_pack + e.offset, e.length};
  return true;
}

bool voicePackFindClip(const char* name, VoiceClip* out) {
  if (!s_pack) return false;
  const int i = voicePackFind(s_pack, s_header, name);
  return i >= 0 && voicePackClipAt((size_t)i, out);
}
