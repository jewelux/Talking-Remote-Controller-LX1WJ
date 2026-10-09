#!/usr/bin/env python3
"""
Build the voice pack (voices.bin) from the voice_*.wav clips.

The pack is flashed into the "voices" partition (see firmware/partitions.csv)
and memory-mapped by the firmware (voice_pack.cpp). Layout, little-endian,
must match firmware/voice_pack_format.h:

  header (32 B)  magic "HTVP", u16 version, u16 count, u32 sample rate,
                 u32 index offset, u32 data offset, u32 total size,
                 u32 CRC32 of the data, u32 reserved
  index          count x (char name[20], u32 offset, u32 length), sorted by name
  data           PCM16 mono clips, each starting 4-byte aligned

The clips are stored without silence around them: leading and trailing zeros
are dropped, and so is a trailing tail quieter than --tail-db (with a short
fade-out where it is cut). The firmware adds the gap between words itself.

Only the standard library is used, so CI can build the pack without a venv.

Usage:
  python build_voice_pack.py
  python build_voice_pack.py --in voice_clips --out voices.bin
Flash:
  esptool --chip esp32s3 -p COM6 write-flash 0x810000 voices.bin
"""

from __future__ import annotations

import argparse
import array
import math
import re
import struct
import wave
import zlib
from pathlib import Path

HERE = Path(__file__).resolve().parent

MAGIC = b"HTVP"
VERSION = 1
HEADER_SIZE = 32
NAME_LEN = 20
ENTRY_SIZE = NAME_LEN + 8
PARTITION_SIZE = 0x7F0000  # the "voices" partition in firmware/partitions.csv

# File names whose clip name differs from the plain stem.
ALIASES = {
    "a_m": "am",
    "thank_you": "thankyou",
    "v_f_o": "vfo",
}


def clip_name(path: Path) -> str:
    """voice_s_meter.wav -> s_meter (the token speakToken() looks up)."""
    base = path.stem.lower()
    base = re.sub(r"[^a-z0-9_]+", "_", base)
    base = re.sub(r"_+", "_", base).strip("_")
    base = base.removeprefix("voice_")
    return ALIASES.get(base, base)


def trim_silence(pcm: bytes, rate: int, tail_db: float) -> bytes:
    """Drop leading/trailing zeros and a trailing tail quieter than tail_db below the peak."""
    s = array.array("h", pcm)
    start, end = 0, len(s)
    while start < end and s[start] == 0:
        start += 1
    while end > start and s[end - 1] == 0:
        end -= 1
    s = s[start:end]
    if not s:
        return b""

    peak = max(abs(x) for x in s)
    threshold = peak * 10 ** (tail_db / 20.0)
    win = rate * 5 // 1000
    keep = len(s)
    while keep >= win:
        w = s[keep - win:keep]
        if math.sqrt(sum(x * x for x in w) / win) >= threshold:
            break
        keep -= win
    if keep < len(s):
        s = s[:keep]
        fade = min(len(s), rate * 3 // 1000)
        for i in range(fade):
            s[len(s) - 1 - i] = int(s[len(s) - 1 - i] * i / fade)
    return s.tobytes()


def load_pcm16_mono(path: Path, rate: int) -> bytes:
    with wave.open(str(path), "rb") as wav_file:
        if wav_file.getnchannels() != 1:
            raise SystemExit(f"{path.name}: not mono")
        if wav_file.getsampwidth() != 2:
            raise SystemExit(f"{path.name}: not 16-bit PCM")
        if wav_file.getframerate() != rate:
            raise SystemExit(f"{path.name}: {wav_file.getframerate()} Hz, expected {rate} Hz")
        return wav_file.readframes(wav_file.getnframes())


def align4(n: int) -> int:
    return (n + 3) & ~3


def build_pack(clips: dict[str, bytes], rate: int) -> bytes:
    names = sorted(clips)
    index_offset = HEADER_SIZE
    data_offset = align4(index_offset + len(names) * ENTRY_SIZE)

    index = bytearray()
    data = bytearray()
    for name in names:
        pcm = clips[name]
        offset = data_offset + len(data)
        index += struct.pack("<20sII", name.encode("ascii"), offset, len(pcm))
        data += pcm
        data += b"\0" * (align4(len(data)) - len(data))

    total = data_offset + len(data)
    header = struct.pack("<4sHHIIIIII", MAGIC, VERSION, len(names), rate, index_offset,
                         data_offset, total, zlib.crc32(data), 0)
    pad = b"\0" * (data_offset - index_offset - len(index))
    return header + bytes(index) + pad + bytes(data)


def main() -> None:
    ap = argparse.ArgumentParser(description="Build voices.bin from voice_*.wav clips.")
    ap.add_argument("--in", dest="in_dir", default=str(HERE / "voice_clips"), help="Folder with voice_*.wav")
    ap.add_argument("--out", default=str(HERE / "voices.bin"), help="Output pack (default: voices.bin here)")
    ap.add_argument("--rate", type=int, default=8000, help="Sample rate of the clips (I2S_SAMPLE_RATE)")
    ap.add_argument("--tail-db", type=float, default=-45.0,
                    help="Cut a trailing tail quieter than this, relative to the peak (default: -45). "
                         "Word-final s, f, z sit around -40 dB, so a higher value cuts them off")
    args = ap.parse_args()

    clips: dict[str, bytes] = {}
    raw_total = 0
    for path in sorted(Path(args.in_dir).glob("voice_*.wav")):
        name = clip_name(path)
        if len(name) >= NAME_LEN:
            raise SystemExit(f"{path.name}: name '{name}' longer than {NAME_LEN - 1} characters")
        if name in clips:
            raise SystemExit(f"{path.name}: duplicate clip name '{name}'")
        raw = load_pcm16_mono(path, args.rate)
        clips[name] = trim_silence(raw, args.rate, args.tail_db)
        if not clips[name]:
            raise SystemExit(f"{path.name}: silent")
        raw_total += len(raw)
    if not clips:
        raise SystemExit(f"No voice_*.wav in {args.in_dir}")

    pack = build_pack(clips, args.rate)
    if len(pack) > PARTITION_SIZE:
        raise SystemExit(f"Pack is {len(pack)} B, the partition holds {PARTITION_SIZE} B")
    Path(args.out).write_bytes(pack)

    pcm_total = sum(len(pcm) for pcm in clips.values())
    print(f"OK: wrote {args.out}")
    print(f"Clips: {len(clips)}  PCM: {pcm_total} B ({pcm_total / args.rate / 2:.1f} s), "
          f"{raw_total - pcm_total} B of silence dropped  "
          f"pack: {len(pack)} B ({len(pack) / 1024:.1f} KiB)")


if __name__ == "__main__":
    main()
