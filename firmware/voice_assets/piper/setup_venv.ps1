# Creates .venv with Piper TTS and downloads the default voice into models/.
param([string]$Voice = "en_US-lessac-medium")

$ErrorActionPreference = "Stop"
Push-Location $PSScriptRoot
try {
  if (-not (Test-Path .venv)) { py -m venv .venv }
  .\.venv\Scripts\python -m pip install --upgrade pip
  .\.venv\Scripts\python -m pip install -r requirements.txt
  .\.venv\Scripts\python -m piper.download_voices $Voice --data-dir models
} finally {
  Pop-Location
}
