#pragma once

#include "radio_globals.h"

// HELP topics. Console help lines go through helpLine() under the right helpHeading().
enum HelpSection : uint8_t { HELP_GENERAL = 1, HELP_RADIO = 2, HELP_DIAG = 4 };
void helpHeading(HelpSection section, const char* title);
void helpLine(const char* text);
// Empty topic: the topic list. GENERAL, RADIO, DIAG, ALL, or a word to search for.
void printHelp(const String& topic);
String readLine();
String readLineFrom(Stream& input, String& lineBuffer);
String upperCopy(String s);
bool processHamtrcServiceCommand(String line, Print& output);
void processCommand(String line);
