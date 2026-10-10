#!/usr/bin/env python3
"""
Interactive Piper TTS try-out: type a phrase, hear it on the default sound device.

By default plays at the voice's native sample rate. Use --rate 8000 to hear the
clip processed exactly like generate_voices.py does for the firmware (trimmed,
resampled, peak-normalized).

Usage (from this folder, after setup_venv.ps1):
  .venv\\Scripts\\python say.py
  .venv\\Scripts\\python say.py --voice en_US-ryan-high --rate 8000
  .venv\\Scripts\\python say.py "v f o a"
"""

from __future__ import annotations

import argparse
import io
import re
import winsound
from pathlib import Path

import soundfile as sf
from piper import PiperVoice, SynthesisConfig

from generate_voices import (DEFAULT_RATE, DEFAULT_VOICE, DEFAULT_VOLUME, HERE, add_synth_args,
                             ensure_voice, make_syn_config, process, synthesize)


def play(audio, sr: int) -> None:
    buf = io.BytesIO()
    sf.write(buf, audio, sr, format="WAV", subtype="PCM_16")
    winsound.PlaySound(buf.getvalue(), winsound.SND_MEMORY)


def say(text: str, voice: PiperVoice, syn_config: SynthesisConfig, args: argparse.Namespace) -> None:
    audio, sr = synthesize(voice, text, syn_config)
    if audio.size == 0:
        print("(no audio)")
        return
    if args.rate:
        audio, sr = process(audio, sr, args), args.rate
    print(f"{len(audio) / sr:5.2f}s @ {sr} Hz")
    if args.save:
        out_dir = Path(args.save)
        out_dir.mkdir(parents=True, exist_ok=True)
        name = re.sub(r"[^a-z0-9_]+", "_", text.lower()).strip("_")[:40] or "clip"
        sf.write(out_dir / f"{name}.wav", audio, sr, subtype="PCM_16")
    play(audio, sr)


def main() -> None:
    ap = argparse.ArgumentParser(description="Type a phrase, hear it spoken by Piper TTS.")
    ap.add_argument("text", nargs="?", help="Say this once and exit (otherwise interactive)")
    ap.add_argument("--voice", default=DEFAULT_VOICE, help=f"Piper voice name (default: {DEFAULT_VOICE})")
    ap.add_argument("--models-dir", default=str(HERE / "models"), help="Where Piper voices are stored")
    ap.add_argument("--rate", type=int, default=None,
                    help=f"Process like the firmware clips at this rate (e.g. {DEFAULT_RATE}); default: native")
    add_synth_args(ap)
    ap.add_argument("--save", default="", help="Also save each utterance as WAV into this folder")
    args = ap.parse_args()
    # process() settings, same defaults as generate_voices.py
    args.lead_ms, args.trail_ms, args.trim_db, args.volume = 0.0, 0.0, -40.0, DEFAULT_VOLUME

    voice = PiperVoice.load(ensure_voice(args.voice, Path(args.models_dir)))
    syn_config = make_syn_config(args)

    if args.text:
        say(args.text, voice, syn_config, args)
        return

    print(f"Voice: {args.voice}. Type a phrase and press Enter; empty line quits.")
    while True:
        try:
            text = input("> ").strip()
        except (EOFError, KeyboardInterrupt):
            print()
            break
        if not text:
            break
        try:
            say(text, voice, syn_config, args)
        except KeyboardInterrupt:
            print()


if __name__ == "__main__":
    main()
