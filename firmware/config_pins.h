#pragma once

#define HAS_VOICE_voice_xiegu 1
#define HAS_VOICE_voice_r 1

#define DEFAULT_VOLUME_LEVEL 1
#define USE_BUTTONS_KEYPAD 1

#define KEYPAD_DEBOUNCE_MS 30
#define KEYPAD_HOLD_MS 700
#define HAVE_MODE_VOICE 1

static const uint32_t PC_BAUD = 115200;

static const uint32_t CIV_BAUD = 9600;
static const int CIV_RX_PIN = 18;
static const int CIV_TX_PIN = 17;

static const int RS232_RX_PIN = 9;
static const int RS232_TX_PIN = 10;

static const uint32_t CAT_BAUD = 19200;
static const int CAT_TX_PIN = 11;
static const int CAT_RX_PIN = 12;
static const bool CAT_TX_INVERT = false;
static const bool CAT_RX_INVERT = false;

static const int SD_CS_PIN = 14;
static const int SD_MOSI_PIN = 35;
static const int SD_MISO_PIN = 37;
static const int SD_SCK_PIN = 36;

static const byte KP_ROW_PINS[4] = {4, 8, 15, 16};
static const byte KP_COL_PINS[4] = {1, 2, 13, 21};

static const int I2S_BCLK_PIN = 5;
static const int I2S_LRCLK_PIN = 6;
static const int I2S_DOUT_PIN = 7;
static const int AMP_SD_PIN = -1;
static const int I2S_SAMPLE_RATE = 11025;
