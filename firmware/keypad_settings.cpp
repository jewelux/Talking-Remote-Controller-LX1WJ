#include "keypad_settings.h"

#include <stddef.h>

namespace {

using S = Setting;
using K = SettingKind;

// One row per Setting, in its order (tests/keypad checks it).
constexpr SettingInfo kSettings[] = {
    // id              kind       centre label
    {S::None,          K::Switch, false, ""},
    {S::Home,          K::Switch, false, ""},
    {S::RfPower,       K::Number, false, "RFPOWER"},
    {S::Mode,          K::List,   false, "MODE"},
    {S::Nr,            K::Switch, false, "NR"},
    {S::NrLevel,       K::Number, false, "NRLEVEL"},
    {S::Nb,            K::Switch, false, "NB"},
    {S::NbLevel,       K::Number, false, "NBLEVEL"},
    {S::Notch,         K::List,   false, "NOTCH"},
    {S::AutoNotch,     K::Switch, false, "ANOTCH"},
    {S::RxLowCut,      K::Number, false, "HPF"},
    {S::RxHighCut,     K::Number, false, "LPF"},
    {S::PbtInner,      K::Number, true,  "PBT1"},
    {S::PbtOuter,      K::Number, true,  "PBT2"},
    {S::FilterSlot,    K::List,   false, "FILWIDTH"},
    {S::FilterShape,   K::List,   false, "FILSHAPE"},
    {S::Dbf,           K::Switch, false, "DBF"},
    {S::BpfWidth,      K::List,   false, "BPF"},
    {S::Agc,           K::List,   false, "AGC"},
    {S::Preamp,        K::List,   false, "PREAMP"},
    {S::Att,           K::List,   false, "ATT"},
    {S::Antenna,       K::List,   false, "ANT"},
    {S::Proc,          K::Switch, false, "PROC"},
    {S::ProcLevel,     K::Number, false, "PROCLEVEL"},
    {S::Vox,           K::Switch, false, "VOX"},
    {S::VoxGain,       K::Number, false, "VOXGAIN"},
    {S::Monitor,       K::Switch, false, "MONITOR"},
    {S::MonitorLevel,  K::Number, false, "MONLEVEL"},
    {S::TxLowCut,      K::Switch, false, "TXLOWCUT"},
    {S::TxEq,          K::List,   false, "MICEQ"},
    {S::TxHighCut,     K::Switch, false, "TXHIGHCUT"},
    {S::MicGain,       K::Number, false, "MICGAIN"},
    {S::VoxDelay,      K::Number, false, "VOXDELAY"},
    {S::Rit,           K::Switch, false, "RIT"},
    {S::RitOffset,     K::Number, true,  "RITOFFSET"},
    {S::RptShift,      K::List,   false, "RPT"},
    {S::ToneType,      K::List,   false, "TONE"},
    {S::Ctcss,         K::List,   false, "CTCSS"},
    {S::Dcs,           K::List,   false, "DCS"},
    {S::Keyer,         K::Switch, false, "KYR"},
    {S::CwSpeed,       K::Number, false, "CWSPEED"},
    {S::BreakIn,       K::List,   false, "BK"},
    {S::Qsk,           K::Number, false, "QSK"},
    {S::Sidetone,      K::Number, false, "SIDETONE"},
    {S::Paddle,        K::List,   false, "PADDLE"},
    {S::CwPitch,       K::Number, false, "CWPITCH"},
    {S::CwWeight,      K::Number, false, "CWWEIGHT"},
    {S::KeyReverse,    K::Switch, false, "KEYREV"},
    {S::CwDelay,       K::Number, false, "CWDELAY"},
    {S::Baud,          K::List,   false, "BAUD"},
    {S::Ft8x7Menu,     K::Number, false, "MENU"},
    {S::Ft8x7Row,      K::Number, false, "ROW"},
    {S::Profile,       K::List,   true,  "PROFILE"},
    {S::SpeechSpeed,   K::List,   true,  "SPEED"},
    {S::Volume,        K::Number, false, "VOLUME"},
};

static_assert(sizeof(kSettings) / sizeof(kSettings[0]) == (size_t)Setting::Count,
              "one kSettings row per Setting");

}  // namespace

const SettingInfo& settingInfo(Setting setting) {
  const size_t i = (size_t)setting;
  return i < (size_t)Setting::Count ? kSettings[i] : kSettings[0];
}
