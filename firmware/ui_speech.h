#pragma once

#include "radio_globals.h"

void initSpeech();
bool playClipProgmem(const uint8_t* data, size_t length);
void playSilenceMs(int ms);
void playDigit(int d);
void speakDigitsAndPoint(const String& s);
void speakSValue(uint8_t sVal);
bool speakToken(const String& token);
bool speakTokens(const char* const* tokens, size_t count, uint16_t gapMs = 60);
bool speakTokenState(const String& token, bool on);
bool speakTokenPercent(const String& token, uint8_t percent);
void speakOk();
void speakError();
void voiceTest();
void speakProfileIdentityFromSlot(uint8_t id, bool withOk);
void speakBootProfile();
void audioAbortNow();
void audioAmpOn();
void audioAmpOff();
void applyVolumeLevel(uint8_t lvl);
void speakVolumeLevel(uint8_t lvl);
void listVoices();
bool playNamedVoice(const String& token);
