#pragma once

#include "radio_globals.h"

void initSpeech();
bool playClipProgmem(const uint8_t* data, size_t length);
void playSilenceMs(int ms);
void playDigit(int d);
void speakDigitsAndPoint(const String& s);
void speakSValue(const SMeterReading& reading);
bool speakToken(const String& token);
bool speakTokens(const char* const* tokens, size_t count, uint16_t gapMs = 60);
// The name said before a value ("power" in "power 50 watts") and the gap after
// it. Verbose off says neither.
bool speakLabel(const String& token);
void speakLabelClip(const uint8_t* data, size_t length);
bool speakTokenState(const String& token, bool on);
bool speakTokenPercent(const String& token, uint8_t percent);
// The "ok" after a changed value ("volume five ok"), with the gap before it.
// Verbose off says neither.
void speakValueOk();
void speakOk();
void speakError();
void speakTimeout();
void speakNotAvailable();
void playBeep();
void voiceTest();
void speakProfileIdentityFromSlot(uint8_t id, bool withOk);
void speakBootProfile();
void audioAbortNow();
// Speech queued between beginTuningSpeech() and endTuningSpeech() is a tuning
// announcement: starting a new one or calling cancelTuningSpeech() drops it,
// even mid-word, while other speech keeps playing.
void beginTuningSpeech();
void endTuningSpeech();
void cancelTuningSpeech();
bool tuningSpeechActive();
uint32_t tuningSpeechEndedMs();
void audioAmpOn();
void audioAmpOff();
void applyVolumeLevel(uint8_t lvl);
void speakVolumeLevel(uint8_t lvl);
void listVoices();
bool playNamedVoice(const String& token);
