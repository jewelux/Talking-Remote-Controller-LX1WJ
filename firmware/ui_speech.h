#pragma once

#include "radio_globals.h"

void initSpeech();
void playSilenceMs(int ms);
void playDigit(int d);
// Digits one by one, "point" for '.' or ',', a short pause for ' '. Followed by
// its unit or "ok" after the usual 60 ms (playSilenceMs(60)).
void speakNumber(const String& s);
// speakNumber and the 250 ms pause that ends a value: it keeps whatever is
// said next (a mode after a frequency, the next announcement) apart from it.
void speakDigitsAndPoint(const String& s);
void speakSValue(const SMeterReading& reading);
bool speakToken(const String& token);
bool speakTokens(const char* const* tokens, size_t count, uint16_t gapMs = 60);
// The name said before a value ("power" in "power 50 watts") and the gap after
// it. Verbose off says neither.
bool speakLabel(const String& token);
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

// How fast the words are spoken; the pauses between them scale along.
enum class SpeechSpeed : uint8_t { Slow = 0, Normal = 1, Fast = 2 };
static constexpr uint8_t SPEECH_SPEED_COUNT = 3;
extern SpeechSpeed g_speechSpeed;
void applySpeechSpeed(SpeechSpeed speed);
// "SLOW", "NORMAL", "FAST": the console word, and the clip name in lower case.
const char* speechSpeedName(SpeechSpeed speed);
bool parseSpeechSpeed(const String& word, SpeechSpeed& out);
// "speed fast"; verbose off says only "fast".
void speakSpeechSpeed();
void listVoices();
bool playNamedVoice(const String& token);
