# Piper voice generator

Regenerates `../voice_clips/voice_*.wav` with [Piper TTS](https://github.com/OHF-Voice/piper1-gpl).
Output format: mono, PCM16, 8000 Hz (the firmware I2S rate), trimmed, peak-normalized to -1 dBFS,
10 ms lead and 60 ms trailing silence.

## Setup (once)

```powershell
.\setup_venv.ps1            # creates .venv, installs requirements, downloads en_US-lessac-high into models/
```

`.venv/` and `models/` are git-ignored.

## Generate

```powershell
.\.venv\Scripts\python generate_voices.py                 # all clips
.\.venv\Scripts\python generate_voices.py --only cw,fm    # just some
.\.venv\Scripts\python generate_voices.py --header        # also merge into firmware/voice_data.h
.\.venv\Scripts\python generate_voices.py --voice en_US-lessac-medium --length-scale 1.1
```

Run `generate_voices.py --help` for all options (sample rate, silence, trim threshold, peak level).

## Phrase list

`voice_phrases.txt`, one clip per line: `phrase [| symbol] [| say]`.
Letters are spelled with spaces (`c w`, `v f o`); the file name is the phrase with spaces
removed (`voice_cw.wav`). Use `say` to fix a mispronunciation, e.g. `digi | | didgy`.
