param(
  [string]$VoicesList = "VoicesList.txt",
  [string]$OutputDir = "voice_clips",
  [string[]]$AudioPaths = @(),
  [int]$TargetSampleRate = 11025,
  [double]$MinSilenceMs = 260,
  [double]$LeadPadMs = 35,
  [double]$TrailPadMs = 70
)

Set-StrictMode -Version Latest
$ErrorActionPreference = "Stop"
$here = Split-Path -Parent $MyInvocation.MyCommand.Path

function Get-VoiceTokens {
  param([string]$Path)

  $tokens = New-Object System.Collections.Generic.List[string]
  foreach ($line in Get-Content $Path) {
    $trimmed = $line.Trim()
    if (-not $trimmed) { continue }
    if ($trimmed -like "http*") { continue }
    if ($trimmed -like "Unbegrenzt*") { continue }
    if ($trimmed -like "Audition-Sound*") { continue }
    if ($trimmed -match '^([^(]+)\(') {
      $tokens.Add($matches[1].Trim())
    }
  }
  return $tokens
}

function Get-SafeTokenName {
  param([string]$Token)

  $aliases = @{
    "a m" = "am"
    "s meter" = "s_meter"
    "thank you" = "thankyou"
    "v f o" = "vfo"
  }

  $safe = $Token.ToLowerInvariant()
  if ($aliases.ContainsKey($safe)) {
    $safe = $aliases[$safe]
  }
  $safe = $safe -replace '[^a-z0-9_]+', '_'
  $safe = $safe -replace '_+', '_'
  $safe = $safe.Trim('_')
  if (-not $safe.StartsWith("voice_")) {
    $safe = "voice_$safe"
  }
  return $safe
}

Add-Type -Path 'C:\Program Files\CONEXANT\Flow\NAudio.dll'
Add-Type -ReferencedAssemblies 'C:\Program Files\CONEXANT\Flow\NAudio.dll' -Path (Join-Path $here 'CachedSampleProvider.cs')

function Read-AudioSamples {
  param([string[]]$AudioPaths)

  $sampleRate = $null
  $samples = New-Object System.Collections.Generic.List[float]
  foreach ($path in $AudioPaths) {
    $reader = [NAudio.Wave.AudioFileReader]::new($path)
    try {
      if ($reader.WaveFormat.Channels -ne 1) {
        throw "Expected mono audio but got $($reader.WaveFormat.Channels) channels in $path"
      }
      if ($null -eq $sampleRate) {
        $sampleRate = $reader.WaveFormat.SampleRate
      } elseif ($sampleRate -ne $reader.WaveFormat.SampleRate) {
        throw "Mismatched sample rate in $path"
      }

      $buffer = New-Object float[] 4096
      while (($read = $reader.Read($buffer, 0, $buffer.Length)) -gt 0) {
        for ($i = 0; $i -lt $read; $i++) {
          $samples.Add($buffer[$i])
        }
      }
    } finally {
      $reader.Dispose()
    }
  }

  return [pscustomobject]@{
    SampleRate = $sampleRate
    Samples = $samples.ToArray()
  }
}

function Find-Segments {
  param(
    [float[]]$Samples,
    [int]$SampleRate,
    [int]$ExpectedCount,
    [double]$MinSilenceMs,
    [double]$LeadPadMs,
    [double]$TrailPadMs
  )

  $windowSize = [Math]::Max(1, [int]($SampleRate * 0.01))
  $minSilenceWindows = [Math]::Max(1, [int]([Math]::Round(($MinSilenceMs / 1000.0) * $SampleRate / $windowSize)))
  $leadPad = [int]([Math]::Round(($LeadPadMs / 1000.0) * $SampleRate))
  $trailPad = [int]([Math]::Round(($TrailPadMs / 1000.0) * $SampleRate))
  $thresholds = @(0.03, 0.025, 0.02, 0.015, 0.012, 0.01, 0.008, 0.006, 0.004)

  foreach ($threshold in $thresholds) {
    $segments = New-Object System.Collections.Generic.List[object]
    $active = $false
    $startIndex = 0
    $silenceWindows = 0
    $lastActiveSample = 0

    for ($windowStart = 0; $windowStart -lt $Samples.Length; $windowStart += $windowSize) {
      $windowEnd = [Math]::Min($Samples.Length, $windowStart + $windowSize)
      $peak = 0.0
      for ($i = $windowStart; $i -lt $windowEnd; $i++) {
        $amp = [Math]::Abs($Samples[$i])
        if ($amp -gt $peak) { $peak = $amp }
      }

      if ($peak -ge $threshold) {
        if (-not $active) {
          $active = $true
          $startIndex = [Math]::Max(0, $windowStart - $leadPad)
        }
        $silenceWindows = 0
        $lastActiveSample = $windowEnd
      } elseif ($active) {
        $silenceWindows++
        if ($silenceWindows -ge $minSilenceWindows) {
          $endIndex = [Math]::Min($Samples.Length, $lastActiveSample + $trailPad)
          if (($endIndex - $startIndex) -gt [int]($SampleRate * 0.06)) {
            $segments.Add([pscustomobject]@{ Start = $startIndex; End = $endIndex })
          }
          $active = $false
          $silenceWindows = 0
        }
      }
    }

    if ($active) {
      $endIndex = [Math]::Min($Samples.Length, $lastActiveSample + $trailPad)
      if (($endIndex - $startIndex) -gt [int]($SampleRate * 0.06)) {
        $segments.Add([pscustomobject]@{ Start = $startIndex; End = $endIndex })
      }
    }

    if ($segments.Count -eq $ExpectedCount) {
      return [pscustomobject]@{
        Threshold = $threshold
        Segments = $segments
      }
    }
  }

  throw "Could not detect exactly $ExpectedCount segments from audio."
}

function Write-VoiceClip {
  param(
    [float[]]$SourceSamples,
    [int]$SourceRate,
    [int]$Start,
    [int]$End,
    [int]$TargetRate,
    [string]$OutputPath
  )

  $length = $End - $Start
  if ($length -le 0) {
    throw "Invalid clip length for $OutputPath"
  }

  $segment = New-Object float[] $length
  [Array]::Copy($SourceSamples, $Start, $segment, 0, $length)

  $waveFormat = [NAudio.Wave.WaveFormat]::CreateIeeeFloatWaveFormat($SourceRate, 1)
  $provider = [NAudio.Wave.SampleProviders.CachedSampleProvider]::new($segment, $waveFormat)
  $resampled = [NAudio.Wave.SampleProviders.WdlResamplingSampleProvider]::new($provider, $TargetRate)
  [NAudio.Wave.WaveFileWriter]::CreateWaveFile16($OutputPath, $resampled)
}

Push-Location $here
try {
  $tokens = Get-VoiceTokens -Path $VoicesList
  if ($tokens.Count -eq 0) {
    throw "No tokens found in $VoicesList"
  }

  if ($AudioPaths.Count -gt 0) {
    $audioFiles = @(
      foreach ($audioPath in $AudioPaths) {
        Get-Item $audioPath
      }
    )
  } else {
    $audioFiles = @(Get-ChildItem *.wav | Sort-Object Name)
    if ($audioFiles.Count -eq 0) {
      $audioFiles = @(Get-ChildItem *.mp3 | Sort-Object Name)
    }
    if ($audioFiles.Count -eq 0) {
      throw "No WAV or MP3 files found in $here"
    }
  }

  $audio = Read-AudioSamples -AudioPaths $audioFiles.FullName
  $segmentation = Find-Segments -Samples $audio.Samples -SampleRate $audio.SampleRate -ExpectedCount $tokens.Count -MinSilenceMs $MinSilenceMs -LeadPadMs $LeadPadMs -TrailPadMs $TrailPadMs

  $outDir = Join-Path $here $OutputDir
  if (-not (Test-Path $outDir)) {
    New-Item -ItemType Directory -Path $outDir | Out-Null
  }

  $seen = @{}
  for ($i = 0; $i -lt $tokens.Count; $i++) {
    $token = $tokens[$i]
    $safe = Get-SafeTokenName -Token $token
    if ($seen.ContainsKey($safe)) { continue }
    $seen[$safe] = $true

    $segment = $segmentation.Segments[$i]
    $outPath = Join-Path $outDir ($safe + ".wav")
    Write-VoiceClip -SourceSamples $audio.Samples -SourceRate $audio.SampleRate -Start $segment.Start -End $segment.End -TargetRate $TargetSampleRate -OutputPath $outPath
    Write-Output ("WROTE {0} <= {1}" -f (Split-Path $outPath -Leaf), $token)
  }

  Write-Output ("SEGMENTS={0} UNIQUE_OUTPUTS={1} THRESHOLD={2}" -f $segmentation.Segments.Count, $seen.Count, $segmentation.Threshold)
} finally {
  Pop-Location
}
