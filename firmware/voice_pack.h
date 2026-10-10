// The voice clips, read from the "voices" flash partition (see voice_pack_format.h).
#pragma once

#include <stddef.h>
#include <stdint.h>

struct VoiceClip {
  const char* name;
  const uint8_t* data;  // PCM16 mono at I2S_SAMPLE_RATE, memory-mapped flash
  size_t len;           // bytes
};

// Finds, checks and maps the pack, and prints the outcome on the console.
// False leaves every clip missing; the rest of the firmware runs without speech.
bool voicePackInit();
bool voicePackReady();
// Prints "OK (<n> clips, <size> bytes)", "MISSING" or "INVALID (<reason>)" and a newline.
void voicePackPrintStatus();
bool voicePackClipAt(size_t i, VoiceClip* out);
bool voicePackFindClip(const char* name, VoiceClip* out);
