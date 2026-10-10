# Piper voice generator

Regenerates `../voice_clips/voice_*.wav` with [Piper TTS](https://github.com/OHF-Voice/piper1-gpl).
Output format: mono, PCM16, 8000 Hz (the firmware I2S rate), trimmed, peak at 0.5 of full scale,
no silence around the word (the firmware adds the gap between words).

## Setup (once)

```powershell
.\setup_venv.ps1            # creates .venv, installs requirements, downloads en_US-lessac-medium into models/
```

`.venv/` and `models/` are git-ignored.

## Generate

```powershell
.\.venv\Scripts\python generate_voices.py                 # all clips
.\.venv\Scripts\python generate_voices.py --only cw,fm    # just some
.\.venv\Scripts\python generate_voices.py --pack          # also build ../voices.bin (flash it, see build_voice_pack.py)
.\.venv\Scripts\python generate_voices.py --voice en_US-lessac-high --length-scale 1.1
```

Defaults are `--noise-scale 0 --noise-w-scale 0`, so clips are repeatable from run to run but a bit flat.
For a livelier voice raise them (the voice's own values are ~0.667 / ~0.8; `0.2` is a compromise), at the
cost of slight run-to-run variation. Same flags work in `say.py`.

Run `generate_voices.py --help` for all options (sample rate, silence, trim threshold, volume).

## Try phrases

Interactive: type a phrase, hear it on the default sound device (missing voices are downloaded).

```powershell
.\.venv\Scripts\python say.py                                       # lessac-medium, native rate
.\.venv\Scripts\python say.py --voice en_US-ryan-high --rate 8000   # hear it as the firmware clip
.\.venv\Scripts\python say.py "v f o a"                             # one-shot
```

## Phrase list

`voice_phrases.txt`, one clip per line: `phrase [| symbol] [| say]`.
Letters are spelled with spaces (`c w`, `v f o`); the file name is the phrase with spaces
removed (`voice_cw.wav`). Use `say` to fix a mispronunciation, e.g. `digi | | didgy`.

`fallback_phrases.txt` holds the one clip built into the firmware ("voice pack missing"), kept
out of the voice pack; the steps to regenerate it are in that file.
