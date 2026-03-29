#include "sd_profile_parser.h"

void profileLoaderAssignIniProfile(StoredProfile& out, const CivProfile& civ, ProtocolType proto, const char* voiceVendor, const char* voiceDigits, const StoredProfile& parsedDefaults);
void profileLoaderPrepareDefaults(StoredProfile& sp, ProtocolType proto);
void profileLoaderCopyCString(char* dst, size_t dstSize, const char* src);

static int parseIniInt(const String& s, int fallback = 0) {
  String t = s;
  t.trim();
  if (!t.length()) return fallback;
  if (t.startsWith("0x") || t.startsWith("0X")) return (int)strtol(t.c_str(), nullptr, 16);
  return t.toInt();
}

static bool parseIniBool(const String& s, bool fallback = false) {
  String t = s;
  t.trim();
  t.toUpperCase();
  if (t == "1" || t == "TRUE" || t == "YES" || t == "ON") return true;
  if (t == "0" || t == "FALSE" || t == "NO" || t == "OFF") return false;
  return fallback;
}

bool initSdProfiles() {
  SPI.begin(SD_SCK_PIN, SD_MISO_PIN, SD_MOSI_PIN, SD_CS_PIN);
  return SD.begin(SD_CS_PIN);
}

bool loadSingleProfileIni(const String& path, StoredProfile& out) {
  File f = SD.open(path.c_str());
  if (!f) return false;

  CivProfile civ = {0x94, "", CIV_BAUD, 1, CIV_RX_PIN, CIV_TX_PIN, true, false};
  ProtocolType proto = PROTO_CIV;
  char tempName[32] = "";
  char voiceVendor[16] = "icom";
  char voiceDigits[16] = "";
  char variant[16] = "";
  StoredProfile sp;
  profileLoaderPrepareDefaults(sp, proto);
  String section;

  while (f.available()) {
    String line = f.readStringUntil('\n');
    line.trim();
    if (!line.length() || line.startsWith("#") || line.startsWith(";")) continue;

    if (line.startsWith("[") && line.endsWith("]")) {
      section = line.substring(1, line.length() - 1);
      section.trim();
      section.toLowerCase();
      continue;
    }

    int eq = line.indexOf('=');
    if (eq < 0) continue;

    String key = line.substring(0, eq);
    String val = line.substring(eq + 1);
    key.trim();
    val.trim();
    key.toLowerCase();

    if (section == "profile") {
      if (key == "name") profileLoaderCopyCString(tempName, sizeof(tempName), val.c_str());
      else if (key == "voice_vendor") profileLoaderCopyCString(voiceVendor, sizeof(voiceVendor), val.c_str());
      else if (key == "voice_digits") profileLoaderCopyCString(voiceDigits, sizeof(voiceDigits), val.c_str());
      else if (key == "variant") profileLoaderCopyCString(variant, sizeof(variant), val.c_str());
    } else if (section == "protocol") {
      String v = val;
      v.trim();
      v.toUpperCase();
      if (key == "type") {
        if (v == "CIV") proto = PROTO_CIV;
        else if (v == "KENWOOD_ASCII") proto = PROTO_KENWOOD_ASCII;
        else if (v == "ELECRAFT_ASCII") proto = PROTO_ELECRAFT_ASCII;
        else if (v == "YAESU_FT8X7") proto = PROTO_YAESU_FT8X7;
        else if (v == "YAESU_FTDX_ASCII") proto = PROTO_YAESU_FTDX_ASCII;
        sp.protocolType = proto;
        profileLoaderPrepareDefaults(sp, proto);
      }
    } else if (section == "connection") {
      if (key == "civ_addr") civ.civAddr = (uint8_t)parseIniInt(val, civ.civAddr);
      else if (key == "baud") civ.baud = (uint32_t)parseIniInt(val, civ.baud);
      else if (key == "uart_num") civ.uartNum = (int8_t)parseIniInt(val, civ.uartNum);
      else if (key == "rx_pin") civ.rxPin = (int8_t)parseIniInt(val, civ.rxPin);
      else if (key == "tx_pin") civ.txPin = (int8_t)parseIniInt(val, civ.txPin);
      else if (key == "tx_invert") civ.txInvert = parseIniBool(val, civ.txInvert);
      else if (key == "rx_invert") civ.rxInvert = parseIniBool(val, civ.rxInvert);
    } else if (section == "capabilities") {
      if (key == "get_freq") sp.caps.getFreq = parseIniBool(val, sp.caps.getFreq);
      else if (key == "set_freq") sp.caps.setFreq = parseIniBool(val, sp.caps.setFreq);
      else if (key == "get_mode") sp.caps.getMode = parseIniBool(val, sp.caps.getMode);
      else if (key == "set_mode") sp.caps.setMode = parseIniBool(val, sp.caps.setMode);
      else if (key == "get_smeter") sp.caps.getSmeter = parseIniBool(val, sp.caps.getSmeter);
      else if (key == "get_power") sp.caps.getPower = parseIniBool(val, sp.caps.getPower);
      else if (key == "get_swr") sp.caps.getSwr = parseIniBool(val, sp.caps.getSwr);
      else if (key == "get_rxtx") sp.caps.getRxTx = parseIniBool(val, sp.caps.getRxTx);
      else if (key == "get_txfreq") sp.caps.getTxFreq = parseIniBool(val, sp.caps.getTxFreq);
      else if (key == "get_nr") sp.caps.getNr = parseIniBool(val, sp.caps.getNr);
      else if (key == "set_nr") sp.caps.setNr = parseIniBool(val, sp.caps.setNr);
      else if (key == "get_nrlevel") sp.caps.getNrLevel = parseIniBool(val, sp.caps.getNrLevel);
      else if (key == "set_nrlevel") sp.caps.setNrLevel = parseIniBool(val, sp.caps.setNrLevel);
      else if (key == "get_nb") sp.caps.getNb = parseIniBool(val, sp.caps.getNb);
      else if (key == "set_nb") sp.caps.setNb = parseIniBool(val, sp.caps.setNb);
      else if (key == "get_nblevel") sp.caps.getNbLevel = parseIniBool(val, sp.caps.getNbLevel);
      else if (key == "set_nblevel") sp.caps.setNbLevel = parseIniBool(val, sp.caps.setNbLevel);
      else if (key == "get_notch") sp.caps.getNotch = parseIniBool(val, sp.caps.getNotch);
      else if (key == "set_notch") sp.caps.setNotch = parseIniBool(val, sp.caps.setNotch);
      else if (key == "get_notch_width") sp.caps.getNotchWidth = parseIniBool(val, sp.caps.getNotchWidth);
      else if (key == "set_notch_width") sp.caps.setNotchWidth = parseIniBool(val, sp.caps.setNotchWidth);
      else if (key == "get_pbt1") sp.caps.getPbtInner = parseIniBool(val, sp.caps.getPbtInner);
      else if (key == "set_pbt1") sp.caps.setPbtInner = parseIniBool(val, sp.caps.setPbtInner);
      else if (key == "get_pbt2") sp.caps.getPbtOuter = parseIniBool(val, sp.caps.getPbtOuter);
      else if (key == "set_pbt2") sp.caps.setPbtOuter = parseIniBool(val, sp.caps.setPbtOuter);
      else if (key == "get_filshape") sp.caps.getFilterShape = parseIniBool(val, sp.caps.getFilterShape);
      else if (key == "set_filshape") sp.caps.setFilterShape = parseIniBool(val, sp.caps.setFilterShape);
      else if (key == "get_filwidth") sp.caps.getFilterWidth = parseIniBool(val, sp.caps.getFilterWidth);
      else if (key == "set_filwidth") sp.caps.setFilterWidth = parseIniBool(val, sp.caps.setFilterWidth);
      else if (key == "get_lock") sp.caps.getDialLock = parseIniBool(val, sp.caps.getDialLock);
      else if (key == "set_lock") sp.caps.setDialLock = parseIniBool(val, sp.caps.setDialLock);
      else if (key == "get_monitor") sp.caps.getMonitor = parseIniBool(val, sp.caps.getMonitor);
      else if (key == "set_monitor") sp.caps.setMonitor = parseIniBool(val, sp.caps.setMonitor);
      else if (key == "get_monlevel") sp.caps.getMonitorLevel = parseIniBool(val, sp.caps.getMonitorLevel);
      else if (key == "set_monlevel") sp.caps.setMonitorLevel = parseIniBool(val, sp.caps.setMonitorLevel);
      else if (key == "get_transceive") sp.caps.getTransceive = parseIniBool(val, sp.caps.getTransceive);
      else if (key == "set_transceive") sp.caps.setTransceive = parseIniBool(val, sp.caps.setTransceive);
      else if (key == "get_tuner") sp.caps.getTuner = parseIniBool(val, sp.caps.getTuner);
      else if (key == "set_tuner") sp.caps.setTuner = parseIniBool(val, sp.caps.setTuner);
      else if (key == "start_tune") sp.caps.startTune = parseIniBool(val, sp.caps.startTune);
      else if (key == "get_vfo") sp.caps.getVfo = parseIniBool(val, sp.caps.getVfo);
      else if (key == "set_vfo") sp.caps.setVfo = parseIniBool(val, sp.caps.setVfo);
      else if (key == "get_vfo_mode") sp.caps.getVfoMode = parseIniBool(val, sp.caps.getVfoMode);
      else if (key == "set_vfo_mode") sp.caps.setVfoMode = parseIniBool(val, sp.caps.setVfoMode);
      else if (key == "get_split") sp.caps.getSplit = parseIniBool(val, sp.caps.getSplit);
      else if (key == "set_split") sp.caps.setSplit = parseIniBool(val, sp.caps.setSplit);
      else if (key == "get_rit") sp.caps.getRit = parseIniBool(val, sp.caps.getRit);
      else if (key == "set_rit") sp.caps.setRit = parseIniBool(val, sp.caps.setRit);
      else if (key == "get_bstack") sp.caps.getBandStack = parseIniBool(val, sp.caps.getBandStack);
    } else if (section == "commands") {
      if (key == "freq_get") profileLoaderCopyCString(sp.ascii.freqGet, sizeof(sp.ascii.freqGet), val.c_str());
      else if (key == "freq_set_format") profileLoaderCopyCString(sp.ascii.freqSetFormat, sizeof(sp.ascii.freqSetFormat), val.c_str());
      else if (key == "mode_get") profileLoaderCopyCString(sp.ascii.modeGet, sizeof(sp.ascii.modeGet), val.c_str());
      else if (key == "mode_set_format") profileLoaderCopyCString(sp.ascii.modeSetFormat, sizeof(sp.ascii.modeSetFormat), val.c_str());
      else if (key == "if_get") profileLoaderCopyCString(sp.ascii.ifGet, sizeof(sp.ascii.ifGet), val.c_str());
      else if (key == "id_get") profileLoaderCopyCString(sp.ascii.idGet, sizeof(sp.ascii.idGet), val.c_str());
      else if (key == "om_get") profileLoaderCopyCString(sp.ascii.omGet, sizeof(sp.ascii.omGet), val.c_str());
      else if (key == "smeter_get") profileLoaderCopyCString(sp.ascii.smeterGet, sizeof(sp.ascii.smeterGet), val.c_str());
      else if (key == "power_get") profileLoaderCopyCString(sp.ascii.powerGet, sizeof(sp.ascii.powerGet), val.c_str());
      else if (key == "swr_get") profileLoaderCopyCString(sp.ascii.swrGet, sizeof(sp.ascii.swrGet), val.c_str());
      else if (key == "nr_get") profileLoaderCopyCString(sp.ascii.nrGet, sizeof(sp.ascii.nrGet), val.c_str());
      else if (key == "nr_on") profileLoaderCopyCString(sp.ascii.nrOnCmd, sizeof(sp.ascii.nrOnCmd), val.c_str());
      else if (key == "nr_off") profileLoaderCopyCString(sp.ascii.nrOffCmd, sizeof(sp.ascii.nrOffCmd), val.c_str());
      else if (key == "nb_get") profileLoaderCopyCString(sp.ascii.nbGet, sizeof(sp.ascii.nbGet), val.c_str());
      else if (key == "nb_on") profileLoaderCopyCString(sp.ascii.nbOnCmd, sizeof(sp.ascii.nbOnCmd), val.c_str());
      else if (key == "nb_off") profileLoaderCopyCString(sp.ascii.nbOffCmd, sizeof(sp.ascii.nbOffCmd), val.c_str());
      else if (key == "preamp_get") profileLoaderCopyCString(sp.ascii.preampGet, sizeof(sp.ascii.preampGet), val.c_str());
      else if (key == "preamp_on") profileLoaderCopyCString(sp.ascii.preampOnCmd, sizeof(sp.ascii.preampOnCmd), val.c_str());
      else if (key == "preamp_off") profileLoaderCopyCString(sp.ascii.preampOffCmd, sizeof(sp.ascii.preampOffCmd), val.c_str());
      else if (key == "agc_get") profileLoaderCopyCString(sp.ascii.agcGet, sizeof(sp.ascii.agcGet), val.c_str());
      else if (key == "agc_fast") profileLoaderCopyCString(sp.ascii.agcFastCmd, sizeof(sp.ascii.agcFastCmd), val.c_str());
      else if (key == "agc_slow") profileLoaderCopyCString(sp.ascii.agcSlowCmd, sizeof(sp.ascii.agcSlowCmd), val.c_str());
      else if (key == "agc_off") profileLoaderCopyCString(sp.ascii.agcOffCmd, sizeof(sp.ascii.agcOffCmd), val.c_str());
      else if (key == "power_state_get") profileLoaderCopyCString(sp.ascii.powerStateGet, sizeof(sp.ascii.powerStateGet), val.c_str());
      else if (key == "power_state_on") profileLoaderCopyCString(sp.ascii.powerStateOnCmd, sizeof(sp.ascii.powerStateOnCmd), val.c_str());
      else if (key == "power_state_off") profileLoaderCopyCString(sp.ascii.powerStateOffCmd, sizeof(sp.ascii.powerStateOffCmd), val.c_str());
      else if (key == "tuner_get") profileLoaderCopyCString(sp.ascii.tunerGet, sizeof(sp.ascii.tunerGet), val.c_str());
      else if (key == "tuner_on") profileLoaderCopyCString(sp.ascii.tunerOnCmd, sizeof(sp.ascii.tunerOnCmd), val.c_str());
      else if (key == "tuner_off") profileLoaderCopyCString(sp.ascii.tunerOffCmd, sizeof(sp.ascii.tunerOffCmd), val.c_str());
      else if (key == "tune_start") profileLoaderCopyCString(sp.ascii.tuneStartCmd, sizeof(sp.ascii.tuneStartCmd), val.c_str());
      else if (key == "notch_get") profileLoaderCopyCString(sp.ascii.notchGet, sizeof(sp.ascii.notchGet), val.c_str());
      else if (key == "notch_on") profileLoaderCopyCString(sp.ascii.notchOnCmd, sizeof(sp.ascii.notchOnCmd), val.c_str());
      else if (key == "notch_off") profileLoaderCopyCString(sp.ascii.notchOffCmd, sizeof(sp.ascii.notchOffCmd), val.c_str());
      else if (key == "lock_get") profileLoaderCopyCString(sp.ascii.lockGet, sizeof(sp.ascii.lockGet), val.c_str());
      else if (key == "lock_on") profileLoaderCopyCString(sp.ascii.lockOnCmd, sizeof(sp.ascii.lockOnCmd), val.c_str());
      else if (key == "lock_off") profileLoaderCopyCString(sp.ascii.lockOffCmd, sizeof(sp.ascii.lockOffCmd), val.c_str());
    } else if (section == "responses") {
      if (key == "freq_prefix") profileLoaderCopyCString(sp.ascii.freqReplyPrefix, sizeof(sp.ascii.freqReplyPrefix), val.c_str());
      else if (key == "mode_prefix") profileLoaderCopyCString(sp.ascii.modeReplyPrefix, sizeof(sp.ascii.modeReplyPrefix), val.c_str());
      else if (key == "if_prefix") profileLoaderCopyCString(sp.ascii.ifReplyPrefix, sizeof(sp.ascii.ifReplyPrefix), val.c_str());
      else if (key == "id_prefix") profileLoaderCopyCString(sp.ascii.idReplyPrefix, sizeof(sp.ascii.idReplyPrefix), val.c_str());
      else if (key == "om_prefix") profileLoaderCopyCString(sp.ascii.omReplyPrefix, sizeof(sp.ascii.omReplyPrefix), val.c_str());
      else if (key == "smeter_prefix") profileLoaderCopyCString(sp.ascii.smeterReplyPrefix, sizeof(sp.ascii.smeterReplyPrefix), val.c_str());
      else if (key == "power_prefix") profileLoaderCopyCString(sp.ascii.powerReplyPrefix, sizeof(sp.ascii.powerReplyPrefix), val.c_str());
      else if (key == "swr_prefix") profileLoaderCopyCString(sp.ascii.swrReplyPrefix, sizeof(sp.ascii.swrReplyPrefix), val.c_str());
      else if (key == "nr_prefix") profileLoaderCopyCString(sp.ascii.nrReplyPrefix, sizeof(sp.ascii.nrReplyPrefix), val.c_str());
      else if (key == "nb_prefix") profileLoaderCopyCString(sp.ascii.nbReplyPrefix, sizeof(sp.ascii.nbReplyPrefix), val.c_str());
      else if (key == "preamp_prefix") profileLoaderCopyCString(sp.ascii.preampReplyPrefix, sizeof(sp.ascii.preampReplyPrefix), val.c_str());
      else if (key == "agc_prefix") profileLoaderCopyCString(sp.ascii.agcReplyPrefix, sizeof(sp.ascii.agcReplyPrefix), val.c_str());
      else if (key == "power_state_prefix") profileLoaderCopyCString(sp.ascii.powerStateReplyPrefix, sizeof(sp.ascii.powerStateReplyPrefix), val.c_str());
      else if (key == "tuner_prefix") profileLoaderCopyCString(sp.ascii.tunerReplyPrefix, sizeof(sp.ascii.tunerReplyPrefix), val.c_str());
      else if (key == "notch_prefix") profileLoaderCopyCString(sp.ascii.notchReplyPrefix, sizeof(sp.ascii.notchReplyPrefix), val.c_str());
      else if (key == "lock_prefix") profileLoaderCopyCString(sp.ascii.lockReplyPrefix, sizeof(sp.ascii.lockReplyPrefix), val.c_str());
    } else if (section == "modes") {
      if (key == "lsb") profileLoaderCopyCString(sp.ascii.modeLsb, sizeof(sp.ascii.modeLsb), val.c_str());
      else if (key == "usb") profileLoaderCopyCString(sp.ascii.modeUsb, sizeof(sp.ascii.modeUsb), val.c_str());
      else if (key == "am") profileLoaderCopyCString(sp.ascii.modeAm, sizeof(sp.ascii.modeAm), val.c_str());
      else if (key == "cw") profileLoaderCopyCString(sp.ascii.modeCw, sizeof(sp.ascii.modeCw), val.c_str());
      else if (key == "rtty") profileLoaderCopyCString(sp.ascii.modeRtty, sizeof(sp.ascii.modeRtty), val.c_str());
      else if (key == "fm") profileLoaderCopyCString(sp.ascii.modeFm, sizeof(sp.ascii.modeFm), val.c_str());
      else if (key == "cwr") profileLoaderCopyCString(sp.ascii.modeCwr, sizeof(sp.ascii.modeCwr), val.c_str());
      else if (key == "rttyr") profileLoaderCopyCString(sp.ascii.modeRttyR, sizeof(sp.ascii.modeRttyR), val.c_str());
      else if (key == "digi") profileLoaderCopyCString(sp.ascii.modeDigi, sizeof(sp.ascii.modeDigi), val.c_str());
    }
  }

  f.close();

  if (!tempName[0]) profileLoaderCopyCString(tempName, sizeof(tempName), path.c_str());
  civ.name = tempName;
  profileLoaderAssignIniProfile(out, civ, proto, voiceVendor, voiceDigits, sp);
  profileLoaderCopyCString(out.variant, sizeof(out.variant), variant);
  return true;
}
