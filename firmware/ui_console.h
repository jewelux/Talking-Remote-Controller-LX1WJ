#pragma once

#include "radio_globals.h"

void printHelp();
String readLine();
String readLineFrom(Stream& input, String& lineBuffer);
String upperCopy(String s);
bool processHamtrcServiceCommand(String line, Print& output);
void processCommand(String line);
