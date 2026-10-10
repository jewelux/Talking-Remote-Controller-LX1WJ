#!/usr/bin/env python3
"""
Check that every word the firmware speaks by name has a clip in voice_clips/,
and list the clips and aliases it never speaks.

The clips are in the voice pack, not in the firmware, so the build cannot catch
a missing clip or a misspelled token; the device would say "error" instead.
This reads the firmware source and checks these tokens against the clips and
the aliases in kVoiceAliases:

  - string literals in the token argument of the speech functions in
    TOKEN_FUNCTIONS, both sides of a ?: included ("on" : "off")
  - the word tables in ui_speech.cpp (static const char* const k...[])
  - the parts of every alias in kVoiceAliases
  - the words of every keypad label in kSpokenLabels

The functions in RUNTIME_TOKEN_FUNCTIONS return words that are spoken, or
spelled, at run time. Their literals are not checked, but a clip they name
counts as used. Other tokens made at run time are not seen, so a clip or
alias listed as unused is a candidate to check, not proof. Letters are never
listed: the firmware spells names letter by letter.

A missing clip fails the check; unused clips and aliases are only listed.
Only the standard library is used, so CI can run it without a venv.

Usage:
  python check_voice_tokens.py
"""

from __future__ import annotations

import argparse
import re
from pathlib import Path

from build_voice_pack import clip_name

HERE = Path(__file__).resolve().parent

# Speech functions whose first argument is a token.
TOKEN_FUNCTIONS = (
    "speakClipToken",
    "speakConsoleTokenOrGap",
    "speakFeatureValue",
    "speakLabel",
    "speakPrompt",
    "speakSignedStepValue",
    "speakToken",
    "speakTokenPercent",
    "speakTokenState",
)

# Functions returning the word for a radio state, which the caller speaks.
RUNTIME_TOKEN_FUNCTIONS = (
    "ft8x7AgcText",
    "ft8x7MicEqText",
    "modeToken",
)

STRING = r'"(?:\\.|[^"\\\n])*"'
CHAR = r"'(?:\\.|[^'\\\n])*'"
COMMENT_OR_LITERAL = re.compile(rf"//[^\n]*|/\*.*?\*/|{STRING}|{CHAR}", re.S)
LITERAL = re.compile(r'"((?:\\.|[^"\\\n])*)"')
BRACKET_OR_LITERAL = re.compile(rf"{STRING}|{CHAR}|[()\[\]{{}},]")
CALL = re.compile(r"\b(?:" + "|".join(TOKEN_FUNCTIONS) + r")\s*\(")
DEFINITION = re.compile(r"\b(?:" + "|".join(RUNTIME_TOKEN_FUNCTIONS) + r")\s*\([^()]*\)\s*\{")
WORD_TABLE = re.compile(r"static const char\* const k\w+\[[^\]]*\]\s*=\s*\{(.*?)\};", re.S)
ALIAS_TABLE = re.compile(r"\bkVoiceAliases\[\]\s*=\s*\{(.*?)\n\};", re.S)
ALIAS = re.compile(r'\{\s*"(\w+)",\s*\{([^}]*)\}')
LABEL_TABLE = re.compile(r"\bkSpokenLabels\[\]\s*=\s*\{(.*?)\n\};", re.S)
LABEL = re.compile(r'\{\s*"[^"]*",\s*("[^"]*")\s*\}')


def blank_comments(src: str) -> str:
    """Comments become spaces, newlines kept, so positions and line numbers stay."""
    def blank(m: re.Match) -> str:
        text = m.group(0)
        return text if text[0] in "\"'" else re.sub(r"[^\n]", " ", text)
    return COMMENT_OR_LITERAL.sub(blank, src)


def enclosed(src: str, start: int, stop_at_comma: bool = False) -> str:
    """src from start, just past an opening bracket, to its closing bracket, or
    to the first comma at that level when stop_at_comma is set."""
    depth = 0
    for m in BRACKET_OR_LITERAL.finditer(src, start):
        c = m.group(0)
        if c in "([{":
            depth += 1
        elif c in ")]}":
            if depth == 0:
                return src[start:m.start()]
            depth -= 1
        elif c == "," and depth == 0 and stop_at_comma:
            return src[start:m.start()]
    return src[start:]


def words(literal: str) -> list[str]:
    """The clip names speakToken() looks up for this text."""
    return literal.strip().lower().replace("-", "_").split()


def main() -> None:
    ap = argparse.ArgumentParser(description="Check that the tokens the firmware speaks have clips.")
    ap.add_argument("--clips", default=str(HERE / "voice_clips"), help="Folder with voice_*.wav")
    ap.add_argument("--src", default=str(HERE.parent), help="Firmware source folder")
    args = ap.parse_args()

    clips = {clip_name(p) for p in Path(args.clips).glob("voice_*.wav")}
    if not clips:
        raise SystemExit(f"No voice_*.wav in {args.clips}")

    spoken: dict[str, tuple[str, int]] = {}  # token -> (file, line) where it is first spoken
    aliases: dict[str, tuple[str, int]] = {}  # alias -> (file, line) where it is defined
    runtime: set[str] = set()  # words from RUNTIME_TOKEN_FUNCTIONS
    src = Path(args.src)
    for path in sorted([*src.glob("*.cpp"), *src.glob("*.h"), *src.glob("*.ino")]):
        text = blank_comments(path.read_text(encoding="utf-8"))

        def where(pos: int) -> tuple[str, int]:
            return path.name, text.count("\n", 0, pos) + 1

        def note(literals: str, pos: int) -> None:
            for lit in LITERAL.finditer(literals):
                for w in words(lit.group(1)):
                    spoken.setdefault(w, where(pos))

        for call in CALL.finditer(text):
            note(enclosed(text, call.end(), stop_at_comma=True), call.start())
        for definition in DEFINITION.finditer(text):
            for lit in LITERAL.finditer(enclosed(text, definition.end())):
                runtime.update(words(lit.group(1)))
        for table in LABEL_TABLE.finditer(text):
            for label in LABEL.finditer(table.group(1)):
                note(label.group(1), table.start(1) + label.start())
        if path.name != "ui_speech.cpp":
            continue
        for table in WORD_TABLE.finditer(text):
            note(table.group(1), table.start(1))
        for table in ALIAS_TABLE.finditer(text):
            for alias in ALIAS.finditer(table.group(1)):
                pos = table.start(1) + alias.start()
                aliases[alias.group(1)] = where(pos)
                note(alias.group(2), pos)

    errors = [(at, f"no clip for '{token}'") for token, at in spoken.items()
              if token not in clips and token not in aliases]
    # speakToken() tries the clip first, so such an alias would never be used.
    errors += [(at, f"alias '{alias}' is also a clip") for alias, at in aliases.items() if alias in clips]
    for (name, line), message in sorted(errors):
        print(f"{name}:{line}: {message}")

    by_name = clips & spoken.keys()
    at_run_time = (clips & runtime) - by_name
    rest = clips - by_name - at_run_time
    spelled = {c for c in rest if len(c) == 1}
    unused = sorted(rest - spelled)
    print(f"{len(clips)} clips: {len(by_name)} spoken by name, {len(at_run_time)} at run time, "
          f"{len(spelled)} letters only spelled, {len(unused)} not spoken")
    if unused:
        print(f"Not spoken: {', '.join(unused)}")
    unused_aliases = sorted(aliases.keys() - spoken.keys() - runtime)
    print(f"{len(aliases)} aliases: {len(aliases) - len(unused_aliases)} spoken by name, "
          f"{len(unused_aliases)} not spoken")
    if unused_aliases:
        print(f"Not spoken: {', '.join(unused_aliases)}")
    if errors:
        raise SystemExit(1)
    print("OK")


if __name__ == "__main__":
    main()
