#!/usr/bin/env python3
"""
(Re)generate the voice_*.wav clips with Piper TTS.

Reads voice_phrases.txt, synthesizes every phrase, trims the silence Piper
adds around it, resamples to the firmware clip format (mono, PCM16, 8000 Hz
by default), normalizes the peak and appends a short trailing silence.

Optionally merges the result into firmware/voice_data.h via build_voice_data.py.

Usage (from this folder, after setup_venv.ps1):
  .venv\\Scripts\\python generate_voices.py
  .venv\\Scripts\\python generate_voices.py --only cw,fm --header
"""

from __future__ import annotations

import argparse
import re
import subprocess
import sys
from math import gcd
from pathlib import Path

import numpy as np
import soundfile as sf
from piper import PiperVoice, SynthesisConfig
from piper.download_voices import download_voice
from scipy.signal import resample_poly

HERE = Path(__file__).resolve().parent
DEFAULT_VOICE = "en_US-lessac-high"
DEFAULT_RATE = 8000  # I2S_SAMPLE_RATE in firmware/config_pins.h


def phrase_to_symbol(phrase: str) -> str:
    """'c w' -> 'cw', 'thank you' -> 'thankyou' (same as VoicesList.txt naming)."""
    return re.sub(r"[^a-z0-9_]+", "", phrase.lower())


def load_phrases(path: Path) -> list[tuple[str, str, str]]:
    """Return (symbol, phrase, say) tuples."""
    entries: list[tuple[str, str, str]] = []
    seen: set[str] = set()
    for lineno, raw in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
        line = raw.split("#", 1)[0].strip()
        if not line:
            continue
        fields = [f.strip() for f in line.split("|")]
        phrase = fields[0]
        symbol = (fields[1] if len(fields) > 1 and fields[1] else phrase_to_symbol(phrase)).lower()
        say = fields[2] if len(fields) > 2 and fields[2] else phrase
        if not re.fullmatch(r"[a-z0-9_]+", symbol):
            raise SystemExit(f"{path.name}:{lineno}: invalid symbol '{symbol}'")
        if symbol in seen:
            raise SystemExit(f"{path.name}:{lineno}: duplicate symbol '{symbol}'")
        seen.add(symbol)
        entries.append((symbol, phrase, say))
    return entries


def ensure_voice(voice: str, models_dir: Path) -> Path:
    model = models_dir / f"{voice}.onnx"
    if not model.exists() or not model.with_suffix(".onnx.json").exists():
        print(f"Downloading voice {voice} into {models_dir} ...")
        models_dir.mkdir(parents=True, exist_ok=True)
        download_voice(voice, models_dir)
    return model


def synthesize(voice: PiperVoice, text: str, syn_config: SynthesisConfig) -> tuple[np.ndarray, int]:
    chunks = []
    sample_rate = voice.config.sample_rate
    for chunk in voice.synthesize(text, syn_config=syn_config):
        chunks.append(chunk.audio_float_array)
        sample_rate = chunk.sample_rate
    if not chunks:
        return np.zeros(0, dtype=np.float32), sample_rate
    return np.concatenate(chunks).astype(np.float32), sample_rate


def trim_silence(audio: np.ndarray, sr: int, threshold_db: float, margin_ms: float) -> np.ndarray:
    """Cut leading/trailing parts quieter than threshold_db relative to the peak."""
    peak = float(np.max(np.abs(audio))) if audio.size else 0.0
    if peak <= 0.0:
        return audio
    win = max(1, int(sr * 0.005))
    n_win = len(audio) // win
    if n_win == 0:
        return audio
    frames = audio[: n_win * win].reshape(n_win, win)
    rms = np.sqrt(np.mean(frames ** 2, axis=1))
    active = np.nonzero(rms >= peak * 10 ** (threshold_db / 20.0))[0]
    if active.size == 0:
        return audio
    margin = int(sr * margin_ms / 1000.0)
    start = max(0, active[0] * win - margin)
    end = min(len(audio), (active[-1] + 1) * win + margin)
    return audio[start:end]


def fade_edges(audio: np.ndarray, sr: int, fade_ms: float) -> np.ndarray:
    n = min(len(audio) // 2, int(sr * fade_ms / 1000.0))
    if n > 0:
        ramp = np.linspace(0.0, 1.0, n, dtype=np.float32)
        audio = audio.copy()
        audio[:n] *= ramp
        audio[-n:] *= ramp[::-1]
    return audio


def resample(audio: np.ndarray, src_sr: int, dst_sr: int) -> np.ndarray:
    if src_sr == dst_sr:
        return audio
    g = gcd(src_sr, dst_sr)
    return resample_poly(audio, dst_sr // g, src_sr // g).astype(np.float32)


def process(audio: np.ndarray, src_sr: int, args: argparse.Namespace) -> np.ndarray:
    audio = trim_silence(audio, src_sr, args.trim_db, margin_ms=5.0)
    audio = resample(audio, src_sr, args.rate)
    peak = float(np.max(np.abs(audio))) if audio.size else 0.0
    if peak > 0.0:
        audio = audio * (10 ** (args.peak_db / 20.0) / peak)
    audio = fade_edges(audio, args.rate, fade_ms=3.0)
    lead = np.zeros(int(args.rate * args.lead_ms / 1000.0), dtype=np.float32)
    trail = np.zeros(int(args.rate * args.trail_ms / 1000.0), dtype=np.float32)
    return np.concatenate([lead, audio, trail])


def main() -> None:
    ap = argparse.ArgumentParser(description="Generate voice_*.wav clips with Piper TTS.")
    ap.add_argument("--phrases", default=str(HERE / "voice_phrases.txt"), help="Phrase list")
    ap.add_argument("--voice", default=DEFAULT_VOICE, help=f"Piper voice name (default: {DEFAULT_VOICE})")
    ap.add_argument("--models-dir", default=str(HERE / "models"), help="Where Piper voices are stored")
    ap.add_argument("--out", default=str(HERE.parent / "voice_clips"), help="Output folder for voice_*.wav")
    ap.add_argument("--only", default="", help="Comma-separated symbols to regenerate (e.g. cw,fm)")
    ap.add_argument("--rate", type=int, default=DEFAULT_RATE, help=f"Output sample rate (default: {DEFAULT_RATE})")
    ap.add_argument("--lead-ms", type=float, default=10.0, help="Leading silence in ms (default: 10)")
    ap.add_argument("--trail-ms", type=float, default=60.0, help="Trailing silence in ms (default: 60)")
    ap.add_argument("--trim-db", type=float, default=-40.0, help="Silence threshold relative to peak (default: -40)")
    ap.add_argument("--peak-db", type=float, default=-1.0, help="Peak level in dBFS (default: -1)")
    ap.add_argument("--length-scale", type=float, default=None, help="Speaking speed; >1 slower, <1 faster")
    ap.add_argument("--speaker", type=int, default=None, help="Speaker id for multi-speaker voices")
    ap.add_argument("--header", action="store_true", help="Also merge the clips into firmware/voice_data.h")
    args = ap.parse_args()

    entries = load_phrases(Path(args.phrases))
    if args.only:
        wanted = {s.strip().lower().removeprefix("voice_") for s in args.only.split(",") if s.strip()}
        unknown = wanted - {e[0] for e in entries}
        if unknown:
            raise SystemExit(f"Unknown symbols: {', '.join(sorted(unknown))}")
        entries = [e for e in entries if e[0] in wanted]

    model = ensure_voice(args.voice, Path(args.models_dir))
    voice = PiperVoice.load(model)
    syn_config = SynthesisConfig(speaker_id=args.speaker, length_scale=args.length_scale)

    out_dir = Path(args.out).resolve()
    out_dir.mkdir(parents=True, exist_ok=True)

    total_bytes = 0
    for symbol, phrase, say in entries:
        audio, src_sr = synthesize(voice, say, syn_config)
        if audio.size == 0:
            raise SystemExit(f"Piper produced no audio for '{say}'")
        clip = process(audio, src_sr, args)
        out_path = out_dir / f"voice_{symbol}.wav"
        sf.write(out_path, clip, args.rate, subtype="PCM_16")
        nbytes = len(clip) * 2
        total_bytes += nbytes
        spoken = phrase if say == phrase else f"{phrase} (say: {say})"
        print(f"{out_path.name:28} {len(clip) / args.rate:5.2f}s {nbytes:7d} B  <= {spoken}")

    print(f"Clips: {len(entries)}  PCM total: {total_bytes} B ({total_bytes / 1024:.1f} KiB)  "
          f"format: mono PCM16 {args.rate} Hz  voice: {args.voice}")

    if args.header:
        header = HERE.parent.parent / "voice_data.h"
        cmd = [
            sys.executable, str(HERE.parent / "build_voice_data.py"),
            "--in", str(out_dir), "--base", str(header), "--out", str(header),
            "--target-sr", str(args.rate),
        ]
        subprocess.run(cmd, check=True)


if __name__ == "__main__":
    main()
