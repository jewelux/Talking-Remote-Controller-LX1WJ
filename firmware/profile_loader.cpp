#include "profile_loader.h"

static void clearStoredProfile(StoredProfile& sp) {
  memset(&sp, 0, sizeof(sp));
}

static void copyCString(char* dst, size_t dstSize, const char* src) {
  if (!dst || dstSize == 0) return;
  if (!src) src = "";
  size_t i = 0;
  for (; i + 1 < dstSize && src[i]; ++i) dst[i] = src[i];
  dst[i] = '\0';
}

static void setAsciiDefaults(StoredProfile& sp) {
  copyCString(sp.ascii.freqGet, sizeof(sp.ascii.freqGet), "FA;");
  copyCString(sp.ascii.freqSetFormat, sizeof(sp.ascii.freqSetFormat), "FA%011llu;");
  copyCString(sp.ascii.modeGet, sizeof(sp.ascii.modeGet), "MD;");
  copyCString(sp.ascii.modeSetFormat, sizeof(sp.ascii.modeSetFormat), "MD%s;");
  copyCString(sp.ascii.vfoAGet, sizeof(sp.ascii.vfoAGet), "");
  copyCString(sp.ascii.vfoASetFormat, sizeof(sp.ascii.vfoASetFormat), "");
  copyCString(sp.ascii.vfoBGet, sizeof(sp.ascii.vfoBGet), "");
  copyCString(sp.ascii.vfoBSetFormat, sizeof(sp.ascii.vfoBSetFormat), "");
  copyCString(sp.ascii.ifGet, sizeof(sp.ascii.ifGet), "");
  copyCString(sp.ascii.idGet, sizeof(sp.ascii.idGet), "");
  copyCString(sp.ascii.omGet, sizeof(sp.ascii.omGet), "");
  copyCString(sp.ascii.smeterGet, sizeof(sp.ascii.smeterGet), "");
  copyCString(sp.ascii.powerGet, sizeof(sp.ascii.powerGet), "");
  copyCString(sp.ascii.swrGet, sizeof(sp.ascii.swrGet), "");
  copyCString(sp.ascii.nrGet, sizeof(sp.ascii.nrGet), "");
  copyCString(sp.ascii.nrOnCmd, sizeof(sp.ascii.nrOnCmd), "");
  copyCString(sp.ascii.nrOffCmd, sizeof(sp.ascii.nrOffCmd), "");
  copyCString(sp.ascii.nbGet, sizeof(sp.ascii.nbGet), "");
  copyCString(sp.ascii.nbOnCmd, sizeof(sp.ascii.nbOnCmd), "");
  copyCString(sp.ascii.nbOffCmd, sizeof(sp.ascii.nbOffCmd), "");
  copyCString(sp.ascii.preampGet, sizeof(sp.ascii.preampGet), "");
  copyCString(sp.ascii.preampOnCmd, sizeof(sp.ascii.preampOnCmd), "");
  copyCString(sp.ascii.preampOffCmd, sizeof(sp.ascii.preampOffCmd), "");
  copyCString(sp.ascii.agcGet, sizeof(sp.ascii.agcGet), "");
  copyCString(sp.ascii.agcFastCmd, sizeof(sp.ascii.agcFastCmd), "");
  copyCString(sp.ascii.agcSlowCmd, sizeof(sp.ascii.agcSlowCmd), "");
  copyCString(sp.ascii.agcOffCmd, sizeof(sp.ascii.agcOffCmd), "");
  copyCString(sp.ascii.powerStateGet, sizeof(sp.ascii.powerStateGet), "");
  copyCString(sp.ascii.powerStateOnCmd, sizeof(sp.ascii.powerStateOnCmd), "");
  copyCString(sp.ascii.powerStateOffCmd, sizeof(sp.ascii.powerStateOffCmd), "");
  copyCString(sp.ascii.tunerGet, sizeof(sp.ascii.tunerGet), "");
  copyCString(sp.ascii.tunerOnCmd, sizeof(sp.ascii.tunerOnCmd), "");
  copyCString(sp.ascii.tunerOffCmd, sizeof(sp.ascii.tunerOffCmd), "");
  copyCString(sp.ascii.tuneStartCmd, sizeof(sp.ascii.tuneStartCmd), "");
  copyCString(sp.ascii.splitGet, sizeof(sp.ascii.splitGet), "");
  copyCString(sp.ascii.splitOnCmd, sizeof(sp.ascii.splitOnCmd), "");
  copyCString(sp.ascii.splitOffCmd, sizeof(sp.ascii.splitOffCmd), "");
  copyCString(sp.ascii.vfoGet, sizeof(sp.ascii.vfoGet), "");
  copyCString(sp.ascii.vfoACmd, sizeof(sp.ascii.vfoACmd), "");
  copyCString(sp.ascii.vfoBCmd, sizeof(sp.ascii.vfoBCmd), "");
  copyCString(sp.ascii.vfoSwapCmd, sizeof(sp.ascii.vfoSwapCmd), "");
  copyCString(sp.ascii.notchGet, sizeof(sp.ascii.notchGet), "");
  copyCString(sp.ascii.notchOnCmd, sizeof(sp.ascii.notchOnCmd), "");
  copyCString(sp.ascii.notchOffCmd, sizeof(sp.ascii.notchOffCmd), "");
  copyCString(sp.ascii.lockGet, sizeof(sp.ascii.lockGet), "");
  copyCString(sp.ascii.lockOnCmd, sizeof(sp.ascii.lockOnCmd), "");
  copyCString(sp.ascii.lockOffCmd, sizeof(sp.ascii.lockOffCmd), "");
  copyCString(sp.ascii.freqReplyPrefix, sizeof(sp.ascii.freqReplyPrefix), "FA");
  copyCString(sp.ascii.modeReplyPrefix, sizeof(sp.ascii.modeReplyPrefix), "MD");
  copyCString(sp.ascii.ifReplyPrefix, sizeof(sp.ascii.ifReplyPrefix), "IF");
  copyCString(sp.ascii.idReplyPrefix, sizeof(sp.ascii.idReplyPrefix), "ID");
  copyCString(sp.ascii.omReplyPrefix, sizeof(sp.ascii.omReplyPrefix), "OM");
  copyCString(sp.ascii.smeterReplyPrefix, sizeof(sp.ascii.smeterReplyPrefix), "SM");
  copyCString(sp.ascii.powerReplyPrefix, sizeof(sp.ascii.powerReplyPrefix), "PC");
  copyCString(sp.ascii.swrReplyPrefix, sizeof(sp.ascii.swrReplyPrefix), "RM");
  copyCString(sp.ascii.nrReplyPrefix, sizeof(sp.ascii.nrReplyPrefix), "");
  copyCString(sp.ascii.nbReplyPrefix, sizeof(sp.ascii.nbReplyPrefix), "");
  copyCString(sp.ascii.preampReplyPrefix, sizeof(sp.ascii.preampReplyPrefix), "");
  copyCString(sp.ascii.agcReplyPrefix, sizeof(sp.ascii.agcReplyPrefix), "");
  copyCString(sp.ascii.powerStateReplyPrefix, sizeof(sp.ascii.powerStateReplyPrefix), "");
  copyCString(sp.ascii.tunerReplyPrefix, sizeof(sp.ascii.tunerReplyPrefix), "");
  copyCString(sp.ascii.splitReplyPrefix, sizeof(sp.ascii.splitReplyPrefix), "");
  copyCString(sp.ascii.vfoReplyPrefix, sizeof(sp.ascii.vfoReplyPrefix), "");
  copyCString(sp.ascii.notchReplyPrefix, sizeof(sp.ascii.notchReplyPrefix), "");
  copyCString(sp.ascii.lockReplyPrefix, sizeof(sp.ascii.lockReplyPrefix), "");
  copyCString(sp.ascii.modeLsb, sizeof(sp.ascii.modeLsb), "1");
  copyCString(sp.ascii.modeUsb, sizeof(sp.ascii.modeUsb), "2");
  copyCString(sp.ascii.modeAm, sizeof(sp.ascii.modeAm), "5");
  copyCString(sp.ascii.modeCw, sizeof(sp.ascii.modeCw), "3");
  copyCString(sp.ascii.modeRtty, sizeof(sp.ascii.modeRtty), "6");
  copyCString(sp.ascii.modeFm, sizeof(sp.ascii.modeFm), "4");
  copyCString(sp.ascii.modeCwr, sizeof(sp.ascii.modeCwr), "7");
  copyCString(sp.ascii.modeRttyR, sizeof(sp.ascii.modeRttyR), "8");
  copyCString(sp.ascii.modeDigi, sizeof(sp.ascii.modeDigi), "6");
}

static void setFt8x7Bank6Defaults(StoredProfile& sp) {
  sp.ft8x7Bank6.repeaterOffsetsHz[0] = 600000UL;
  sp.ft8x7Bank6.repeaterOffsetsHz[1] = 7600000UL;
  sp.ft8x7Bank6.ctcssDefaultTenths = 885;
  sp.ft8x7Bank6.dcsDefaultCode = 23;
}

static void setProtocolDefaults(StoredProfile& sp) {
  sp.caps.getFreq = true;
  sp.caps.setFreq = true;
  sp.caps.getMode = true;
  sp.caps.setMode = true;
  sp.caps.getSmeter = false;
  sp.caps.getPower = false;
  sp.caps.getSwr = false;
  sp.caps.getRxTx = false;
  sp.caps.getTxFreq = false;
  sp.caps.getNr = false;
  sp.caps.setNr = false;
  sp.caps.getNrLevel = false;
  sp.caps.setNrLevel = false;
  sp.caps.getNb = false;
  sp.caps.setNb = false;
  sp.caps.getNbLevel = false;
  sp.caps.setNbLevel = false;
  sp.caps.getNotch = false;
  sp.caps.setNotch = false;
  sp.caps.getNotchWidth = false;
  sp.caps.setNotchWidth = false;
  sp.caps.getPbtInner = false;
  sp.caps.setPbtInner = false;
  sp.caps.getPbtOuter = false;
  sp.caps.setPbtOuter = false;
  sp.caps.getFilterShape = false;
  sp.caps.setFilterShape = false;
  sp.caps.getFilterWidth = false;
  sp.caps.setFilterWidth = false;
  sp.caps.getDialLock = false;
  sp.caps.setDialLock = false;
  sp.caps.getMonitor = false;
  sp.caps.setMonitor = false;
  sp.caps.getMonitorLevel = false;
  sp.caps.setMonitorLevel = false;
  sp.caps.getTransceive = false;
  sp.caps.setTransceive = false;
  sp.caps.getTuner = false;
  sp.caps.setTuner = false;
  sp.caps.startTune = false;
  sp.caps.getVfo = false;
  sp.caps.setVfo = false;
  sp.caps.getVfoMode = false;
  sp.caps.setVfoMode = false;
  sp.caps.getSplit = false;
  sp.caps.setSplit = false;
  sp.caps.getRit = false;
  sp.caps.setRit = false;
  sp.caps.getBandStack = false;
  setAsciiDefaults(sp);
  setFt8x7Bank6Defaults(sp);

  if (sp.protocolType == PROTO_CIV) {
    sp.caps.getSmeter = true;
    sp.caps.getPower = true;
    sp.caps.getSwr = true;
  } else if (sp.protocolType == PROTO_KENWOOD_ASCII) {
    copyCString(sp.ascii.smeterGet, sizeof(sp.ascii.smeterGet), "SM0;");
    copyCString(sp.ascii.swrGet, sizeof(sp.ascii.swrGet), "RM1;");
    sp.caps.getSmeter = true;
    sp.caps.getSwr = true;
  } else if (sp.protocolType == PROTO_ELECRAFT_ASCII) {
    copyCString(sp.ascii.ifGet, sizeof(sp.ascii.ifGet), "IF;");
    copyCString(sp.ascii.idGet, sizeof(sp.ascii.idGet), "ID;");
    copyCString(sp.ascii.omGet, sizeof(sp.ascii.omGet), "OM;");
    copyCString(sp.ascii.smeterGet, sizeof(sp.ascii.smeterGet), "SM;");
    copyCString(sp.ascii.powerGet, sizeof(sp.ascii.powerGet), "PO;");
    copyCString(sp.ascii.swrGet, sizeof(sp.ascii.swrGet), "SW;");
    copyCString(sp.ascii.nbGet, sizeof(sp.ascii.nbGet), "NB;");
    copyCString(sp.ascii.nbOnCmd, sizeof(sp.ascii.nbOnCmd), "NB1;");
    copyCString(sp.ascii.nbOffCmd, sizeof(sp.ascii.nbOffCmd), "NB0;");
    copyCString(sp.ascii.preampGet, sizeof(sp.ascii.preampGet), "PA;");
    copyCString(sp.ascii.preampOnCmd, sizeof(sp.ascii.preampOnCmd), "PA1;");
    copyCString(sp.ascii.preampOffCmd, sizeof(sp.ascii.preampOffCmd), "PA0;");
    copyCString(sp.ascii.agcGet, sizeof(sp.ascii.agcGet), "GT;");
    copyCString(sp.ascii.agcFastCmd, sizeof(sp.ascii.agcFastCmd), "GT002;");
    copyCString(sp.ascii.agcSlowCmd, sizeof(sp.ascii.agcSlowCmd), "GT004;");
    copyCString(sp.ascii.powerStateGet, sizeof(sp.ascii.powerStateGet), "PS;");
    copyCString(sp.ascii.powerStateOnCmd, sizeof(sp.ascii.powerStateOnCmd), "PS1;");
    copyCString(sp.ascii.powerStateOffCmd, sizeof(sp.ascii.powerStateOffCmd), "PS0;");
    copyCString(sp.ascii.powerReplyPrefix, sizeof(sp.ascii.powerReplyPrefix), "PO");
    copyCString(sp.ascii.swrReplyPrefix, sizeof(sp.ascii.swrReplyPrefix), "SW");
    copyCString(sp.ascii.nbReplyPrefix, sizeof(sp.ascii.nbReplyPrefix), "NB");
    copyCString(sp.ascii.preampReplyPrefix, sizeof(sp.ascii.preampReplyPrefix), "PA");
    copyCString(sp.ascii.agcReplyPrefix, sizeof(sp.ascii.agcReplyPrefix), "GT");
    copyCString(sp.ascii.powerStateReplyPrefix, sizeof(sp.ascii.powerStateReplyPrefix), "PS");
    sp.caps.getSmeter = true;
    sp.caps.getPower = true;
    sp.caps.getSwr = true;
    sp.caps.getNb = true;
    sp.caps.setNb = true;
    copyCString(sp.ascii.modeFm, sizeof(sp.ascii.modeFm), "4");
    copyCString(sp.ascii.modeRtty, sizeof(sp.ascii.modeRtty), "6");
    copyCString(sp.ascii.modeDigi, sizeof(sp.ascii.modeDigi), "6");
  } else if (sp.protocolType == PROTO_YAESU_FTDX_ASCII) {
    copyCString(sp.ascii.freqSetFormat, sizeof(sp.ascii.freqSetFormat), "FA%09llu;");
    copyCString(sp.ascii.smeterGet, sizeof(sp.ascii.smeterGet), "SM0;");
    copyCString(sp.ascii.powerGet, sizeof(sp.ascii.powerGet), "RM5;");
    copyCString(sp.ascii.swrGet, sizeof(sp.ascii.swrGet), "RM6;");
    copyCString(sp.ascii.vfoAGet, sizeof(sp.ascii.vfoAGet), "FA;");
    copyCString(sp.ascii.vfoASetFormat, sizeof(sp.ascii.vfoASetFormat), "FA%09llu;");
    copyCString(sp.ascii.vfoBGet, sizeof(sp.ascii.vfoBGet), "FB;");
    copyCString(sp.ascii.vfoBSetFormat, sizeof(sp.ascii.vfoBSetFormat), "FB%09llu;");
    copyCString(sp.ascii.smeterReplyPrefix, sizeof(sp.ascii.smeterReplyPrefix), "SM0");
    copyCString(sp.ascii.powerReplyPrefix, sizeof(sp.ascii.powerReplyPrefix), "RM5");
    copyCString(sp.ascii.swrReplyPrefix, sizeof(sp.ascii.swrReplyPrefix), "RM6");
    copyCString(sp.ascii.tunerGet, sizeof(sp.ascii.tunerGet), "AC;");
    copyCString(sp.ascii.tunerOnCmd, sizeof(sp.ascii.tunerOnCmd), "AC001;");
    copyCString(sp.ascii.tunerOffCmd, sizeof(sp.ascii.tunerOffCmd), "AC000;");
    copyCString(sp.ascii.tuneStartCmd, sizeof(sp.ascii.tuneStartCmd), "AC002;");
    copyCString(sp.ascii.tunerReplyPrefix, sizeof(sp.ascii.tunerReplyPrefix), "AC");
    copyCString(sp.ascii.splitGet, sizeof(sp.ascii.splitGet), "ST;");
    copyCString(sp.ascii.splitOnCmd, sizeof(sp.ascii.splitOnCmd), "ST1;");
    copyCString(sp.ascii.splitOffCmd, sizeof(sp.ascii.splitOffCmd), "ST0;");
    copyCString(sp.ascii.splitReplyPrefix, sizeof(sp.ascii.splitReplyPrefix), "ST");
    copyCString(sp.ascii.vfoGet, sizeof(sp.ascii.vfoGet), "VS;");
    copyCString(sp.ascii.vfoACmd, sizeof(sp.ascii.vfoACmd), "VS0;");
    copyCString(sp.ascii.vfoBCmd, sizeof(sp.ascii.vfoBCmd), "VS1;");
    copyCString(sp.ascii.vfoSwapCmd, sizeof(sp.ascii.vfoSwapCmd), "SV;");
    copyCString(sp.ascii.vfoReplyPrefix, sizeof(sp.ascii.vfoReplyPrefix), "VS");
    copyCString(sp.ascii.lockGet, sizeof(sp.ascii.lockGet), "LK;");
    copyCString(sp.ascii.lockOnCmd, sizeof(sp.ascii.lockOnCmd), "LK1;");
    copyCString(sp.ascii.lockOffCmd, sizeof(sp.ascii.lockOffCmd), "LK0;");
    copyCString(sp.ascii.lockReplyPrefix, sizeof(sp.ascii.lockReplyPrefix), "LK");
    copyCString(sp.ascii.modeLsb, sizeof(sp.ascii.modeLsb), "1");
    copyCString(sp.ascii.modeUsb, sizeof(sp.ascii.modeUsb), "2");
    copyCString(sp.ascii.modeCw, sizeof(sp.ascii.modeCw), "3");
    copyCString(sp.ascii.modeFm, sizeof(sp.ascii.modeFm), "4");
    copyCString(sp.ascii.modeAm, sizeof(sp.ascii.modeAm), "5");
    copyCString(sp.ascii.modeRtty, sizeof(sp.ascii.modeRtty), "6");
    copyCString(sp.ascii.modeCwr, sizeof(sp.ascii.modeCwr), "7");
    copyCString(sp.ascii.modeDigi, sizeof(sp.ascii.modeDigi), "8");
    copyCString(sp.ascii.modeRttyR, sizeof(sp.ascii.modeRttyR), "9");
    sp.caps.getSmeter = true;
    sp.caps.getPower = true;
    sp.caps.getSwr = true;
    sp.caps.getDialLock = true;
    sp.caps.setDialLock = true;
    sp.caps.getTuner = true;
    sp.caps.setTuner = true;
    sp.caps.startTune = true;
    sp.caps.getVfo = true;
    sp.caps.setVfo = true;
    sp.caps.getSplit = true;
    sp.caps.setSplit = true;
  }
}

static void assignStoredProfile(StoredProfile& sp, const CivProfile& civ, ProtocolType proto, const char* voiceVendor, const char* voiceDigits, bool fromSd) {
  clearStoredProfile(sp);
  sp.civ = civ;
  sp.protocolType = proto;
  setProtocolDefaults(sp);
  if (proto == PROTO_CIV && civ.civAddr == 0x94) {
    sp.caps.getRxTx = true;
    sp.caps.getTxFreq = true;
    sp.caps.getNr = true;
    sp.caps.setNr = true;
    sp.caps.getNrLevel = true;
    sp.caps.setNrLevel = true;
    sp.caps.getNb = true;
    sp.caps.setNb = true;
    sp.caps.getNbLevel = true;
    sp.caps.setNbLevel = true;
    sp.caps.getNotch = true;
    sp.caps.setNotch = true;
    sp.caps.getNotchWidth = true;
    sp.caps.setNotchWidth = true;
    sp.caps.getPbtInner = true;
    sp.caps.setPbtInner = true;
    sp.caps.getPbtOuter = true;
    sp.caps.setPbtOuter = true;
    sp.caps.getFilterShape = true;
    sp.caps.setFilterShape = true;
    sp.caps.getFilterWidth = true;
    sp.caps.setFilterWidth = true;
    sp.caps.getDialLock = true;
    sp.caps.setDialLock = true;
    sp.caps.getMonitor = true;
    sp.caps.setMonitor = true;
    sp.caps.getMonitorLevel = true;
    sp.caps.setMonitorLevel = true;
    sp.caps.getTransceive = true;
    sp.caps.setTransceive = true;
    sp.caps.getTuner = true;
    sp.caps.setTuner = true;
    sp.caps.startTune = true;
    sp.caps.getVfo = true;
    sp.caps.setVfo = true;
    sp.caps.getVfoMode = true;
    sp.caps.setVfoMode = true;
    sp.caps.getSplit = true;
    sp.caps.setSplit = true;
    sp.caps.getRit = true;
    sp.caps.setRit = true;
    sp.caps.getBandStack = true;
  }
  sp.valid = true;
  sp.fromSd = fromSd;
  copyCString(sp.name, sizeof(sp.name), civ.name ? civ.name : "");
  copyCString(sp.voiceVendor, sizeof(sp.voiceVendor), voiceVendor ? voiceVendor : "");
  copyCString(sp.voiceDigits, sizeof(sp.voiceDigits), voiceDigits ? voiceDigits : "");
  copyCString(sp.variant, sizeof(sp.variant), "");
  sp.civ.name = sp.name;
}

void seedBuiltInSlots() {
  const CivProfile profile7300 = {0x94, "IC-7300", CIV_BAUD, 1, CIV_RX_PIN, CIV_TX_PIN, true, false};
  const CivProfile profile706 = {0x58, "Icom 706", CIV_BAUD, 1, CIV_RX_PIN, CIV_TX_PIN, true, false};
  const CivProfile profile7300Rs232 = {0x94, "Icom 7300 rs232", CIV_BAUD, 2, RS232_RX_PIN, RS232_TX_PIN, false, false};
  const CivProfile profileG106 = {0x76, "Xiegu 106", CAT_BAUD, 2, CAT_RX_PIN, CAT_TX_PIN, CAT_TX_INVERT, CAT_RX_INVERT};

  for (uint8_t i = 0; i < MAX_PROFILE_SLOTS; ++i) clearStoredProfile(g_slotProfiles[i]);
  assignStoredProfile(g_slotProfiles[0], profile7300, PROTO_CIV, "icom", "7300", false);
  assignStoredProfile(g_slotProfiles[1], profile706, PROTO_CIV, "icom", "706", false);
  assignStoredProfile(g_slotProfiles[2], profile7300Rs232, PROTO_CIV, "icom", "7300232", false);
  assignStoredProfile(g_slotProfiles[3], profileG106, PROTO_CIV, "xiegu", "106", false);
}

void profileLoaderAssignIniProfile(
  StoredProfile& out,
  const CivProfile& civ,
  ProtocolType proto,
  const char* voiceVendor,
  const char* voiceDigits,
  const StoredProfile& parsedDefaults
) {
  assignStoredProfile(out, civ, proto, voiceVendor, voiceDigits, true);
  out.caps = parsedDefaults.caps;
  out.ascii = parsedDefaults.ascii;
  out.ft8x7Bank6 = parsedDefaults.ft8x7Bank6;
}

void profileLoaderPrepareDefaults(StoredProfile& sp, ProtocolType proto) {
  clearStoredProfile(sp);
  sp.protocolType = proto;
  setProtocolDefaults(sp);
}

void profileLoaderCopyCString(char* dst, size_t dstSize, const char* src) {
  copyCString(dst, dstSize, src);
}
