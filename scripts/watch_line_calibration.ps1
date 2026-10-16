param(
    [string]$ProjectRoot = (Split-Path -Parent $PSScriptRoot),
    [ValidateRange(10, 600)][int]$DurationSeconds = 45,
    [ValidateRange(250, 10000)][int]$IntervalMs = 500,
    [string]$LogPath,
    [string]$ReplayFile
)

$ErrorActionPreference = 'Stop'
$reader = Join-Path $ProjectRoot 'scripts\read_line_sensor.ps1'
if (-not $ReplayFile -and -not (Test-Path -LiteralPath $reader)) {
    throw "Reader not found: $reader"
}
if (-not $LogPath) {
    $logDir = Join-Path $ProjectRoot 'logs'
    $LogPath = Join-Path $logDir ('line-calibration-' + (Get-Date -Format 'yyyyMMdd-HHmmss-fff') + '.jsonl')
}
$parent = Split-Path -Parent ([System.IO.Path]::GetFullPath($LogPath))
New-Item -ItemType Directory -Force -Path $parent | Out-Null
if (Test-Path -LiteralPath $LogPath) { throw "Log already exists; choose another LogPath: $LogPath" }
$previousUptime = $null
$previousState = $null
$lastResult = $null
$reads = 0
$restarts = 0
$readErrors = 0
$replayIndex = 0
$replay = if ($ReplayFile) { @(Get-Content -LiteralPath $ReplayFile -Raw | ConvertFrom-Json) } else { @() }
$timer = [System.Diagnostics.Stopwatch]::StartNew()

Write-Host 'Watching calibration. Read-only HOTPLUG: no reset, download, halt or motor command.'
Write-Host 'Park on white. Hold USER >=6 seconds and RELEASE. Remain still on white for 1 second, then sweep black/white for 10 seconds.'
Write-Host 'Keep power on. Do not press RESET or flash firmware during this observation. Ctrl+C stops watching.'
Write-Host 'Expected: NotLearned -> WhiteReference -> Sweeping -> Ready or Failed.'
Write-Host "Log: $LogPath"
try {
    while ($timer.Elapsed.TotalSeconds -lt $DurationSeconds) {
        if ($ReplayFile -and $replayIndex -ge $replay.Count) { break }
        $stamp = Get-Date -Format 'HH:mm:ss.fff'
        try {
            if ($ReplayFile) {
                $r = $replay[$replayIndex]
                $replayIndex++
            } else {
                $r = & $reader -ProjectRoot $ProjectRoot -AsJson | ConvertFrom-Json
            }
            if ($null -eq $r.Uptime_ms -or $null -eq $r.Calibration) { throw 'Incomplete reader output.' }
            $uptime = [uint64]$r.Uptime_ms
            $restart = $false
            if ($null -ne $previousUptime -and $uptime -lt $previousUptime) {
                # Distinguish ordinary 32-bit millis rollover from a restart.
                $rollover = $previousUptime -ge 0xF0000000L -and $uptime -le 0x0FFFFFFFL
                $restart = -not $rollover
                if ($restart) {
                    $restarts++
                    Write-Host ("*** UPTIME_RESTART: {0} -> {1} ms; previous calibration: {2} ***" -f $previousUptime, $uptime, $previousState) -ForegroundColor Yellow
                }
            }
            $entry = [ordered]@{ Time = $stamp; Uptime_restart = $restart; Snapshot = $r }
            $entry | ConvertTo-Json -Depth 6 -Compress | Add-Content -LiteralPath $LogPath -Encoding UTF8
            Write-Host ("{0} up={1,8}ms  {2,-14} white={3,2} sweep={4,3} remaining={5,5}ms fresh={6} saved={7} error={8} {9}" -f
                $stamp, $uptime, $r.Calibration, $r.White_reference_samples, $r.Calibration_samples,
                $r.Calibration_remaining_ms, $r.Fresh_analog_frame, $r.Saved_to_flash, $r.Calibration_error, $r.Failed_channels)
            $previousUptime = $uptime
            $previousState = $r.Calibration
            $lastResult = $r
            $reads++
        } catch {
            $readErrors++
            $message = $_.Exception.Message
            [ordered]@{ Time = $stamp; Read_error = $message } | ConvertTo-Json -Compress |
                Add-Content -LiteralPath $LogPath -Encoding UTF8
            Write-Host "$stamp READ_ERROR: $message" -ForegroundColor Yellow
        }
        if (-not $ReplayFile -and $timer.Elapsed.TotalSeconds -lt $DurationSeconds) {
            Start-Sleep -Milliseconds $IntervalMs
        }
    }
} finally {
    $timer.Stop()
    Write-Host "Finished: reads=$reads uptime_restarts=$restarts read_errors=$readErrors"
    if ($null -ne $lastResult) {
        $lastResult | Select-Object Firmware_diagnostic_version, Calibration, Calibration_error,
            Calibration_valid, Saved_to_flash, Failed_channels, White_reference_samples,
            Calibration_samples, Sweep_coverage_ms, Uptime_ms | Format-List
        if ($lastResult.Calibration -eq 'Failed') {
            $lastResult.Channels | Select-Object Channel, Latest_white_median, Latest_white_spread | Format-Table -AutoSize
        }
    }
    Write-Host "Full record: $LogPath"
}
