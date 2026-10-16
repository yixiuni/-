param(
    [string]$ProjectRoot = (Split-Path -Parent $PSScriptRoot),
    [string]$DumpFile,
    [switch]$AsJson,
    [switch]$Watch,
    [ValidateRange(5, 600)][int]$DurationSeconds = 45,
    [string]$LogPath
)
$ErrorActionPreference = 'Stop'
if ($Watch) {
    if ($DumpFile -or $AsJson) { throw 'Watch cannot be combined with DumpFile or AsJson.' }
    if (-not $LogPath) { $LogPath = Join-Path $ProjectRoot ('logs\line-follow-' + (Get-Date -Format 'yyyyMMdd-HHmmss-fff') + '.jsonl') }
    New-Item -ItemType Directory -Force -Path (Split-Path -Parent ([System.IO.Path]::GetFullPath($LogPath))) | Out-Null
    if (Test-Path -LiteralPath $LogPath) { throw "Log already exists: $LogPath" }
    Write-Host 'Read-only observation. Align the sensor over the START stripe, then SHORT press USER to start/stop.'
    Write-Host 'Hold 2-6s/release = laps; double click = speed; hold >=6s/release = calibration.'
    Write-Host 'The watcher ending does not stop the car. Short press USER to stop.'
    Write-Host "Log: $LogPath"
    $timer = [System.Diagnostics.Stopwatch]::StartNew()
    $last = $null
    try {
        while ($timer.Elapsed.TotalSeconds -lt $DurationSeconds) {
            $stamp = Get-Date -Format 'HH:mm:ss.fff'
            try {
                $r = & $PSCommandPath -ProjectRoot $ProjectRoot -AsJson | ConvertFrom-Json
                [ordered]@{ Time = $stamp; Snapshot = $r } | ConvertTo-Json -Depth 4 -Compress | Add-Content -LiteralPath $LogPath -Encoding UTF8
                Write-Host ("{0} {1,-13} stop={2,-17} pos={3,5} target={4,4}/{5,-4} speed={6,4}/{7,-4} PWM={8,2}/{9,-2} pending={10}:{11}ms mask={12}/{13}" -f
                    $stamp, $r.State, $r.Stop_reason, $r.Position_left_negative, $r.Left_target_cps, $r.Right_target_cps,
                    $r.Left_measured_cps, $r.Right_measured_cps, $r.Left_PWM_percent, $r.Right_PWM_percent,
                    $r.Pending_reason, $r.Pending_ms, $r.Black_mask_hex, $r.Selected_mask_hex)
                $last = $r
            } catch {
                $message = $_.Exception.Message
                [ordered]@{ Time = $stamp; Read_error = $message } | ConvertTo-Json -Compress | Add-Content -LiteralPath $LogPath -Encoding UTF8
                Write-Host "$stamp READ_ERROR: $message" -ForegroundColor Yellow
            }
            if ($timer.Elapsed.TotalSeconds -lt $DurationSeconds) { Start-Sleep -Milliseconds 500 }
        }
    } finally {
        $timer.Stop()
        if ($null -ne $last) { $last | Format-List }
        Write-Host "Full record: $LogPath"
    }
    return
}

if ($DumpFile) {
    $dumpLines = Get-Content -LiteralPath $DumpFile
} else {
    $mapPath = Join-Path $ProjectRoot 'build\Debug\C10B_Test.map'
    if (-not (Test-Path -LiteralPath $mapPath)) { throw 'Build Debug first: map file is missing.' }
    $symbol = [regex]::Match((Get-Content -LiteralPath $mapPath -Raw), '(?m)^\s+0x([0-9a-fA-F]+)\s+line_follow\s*$')
    if (-not $symbol.Success) { throw 'Build the new line-follow firmware first: map has no line_follow symbol.' }
    $address = '0x' + $symbol.Groups[1].Value
    $programmer = Join-Path $env:LOCALAPPDATA 'stm32cube\bundles\programmer\2.23.0\bin\STM32_Programmer_CLI.exe'
    if (-not (Test-Path -LiteralPath $programmer)) { throw "Programmer not found: $programmer" }
    $dumpLines = & $programmer -c port=SWD mode=HOTPLUG -r32 $address 0x80 2>&1
    if ($LASTEXITCODE -ne 0) { $dumpLines | Out-Host; throw 'ST-LINK read failed. End any active debugging session.' }
}
$words = [System.Collections.Generic.List[uint32]]::new()
foreach ($line in $dumpLines) {
    $m = [regex]::Match([string]$line, '^\s*0x[0-9a-fA-F]+\s*:\s*((?:[0-9a-fA-F]{8}(?:\s+|$))+)')
    if ($m.Success) {
        foreach ($word in ($m.Groups[1].Value.Trim() -split '\s+')) { $words.Add([Convert]::ToUInt32($word, 16)) }
    }
}
if ($words.Count -ne 32) { throw "Incomplete line-follow record: $($words.Count)/32 words." }
if ($words[0] -ne 0x4C465731 -or $words[1] -ne 3) {
    throw ('Line-follow record mismatch: magic=0x{0:X8}, version={1}; expected 0x4C465731/version 3. Rebuild and flash the matching Debug ELF.' -f $words[0],$words[1])
}
$states = @('Idle', 'Following', 'Stopped', 'Fault', 'StartRejected')
$reasons = @('None', 'USER', 'NotCalibrated', 'SensorStale', 'NoLine', 'AmbiguousLine', 'SpeedFault', 'Calibrating', 'LapsComplete', 'MarkerTimeout', 'NeedStartMarker','DistanceNotCalibrated')
$faults = @('None', 'LeftNoFeedback', 'RightNoFeedback', 'ReversedFeedback', 'EncoderSampleTimeout', 'BadTarget')
if ($words[2] -ge $states.Count -or $words[3] -ge $reasons.Count -or $words[20] -ge $faults.Count -or $words[27] -ge $reasons.Count -or $words[31] -ge $reasons.Count) { throw 'Invalid diagnostic state.' }
function Signed32([uint32]$v) { [BitConverter]::ToInt32([BitConverter]::GetBytes($v), 0) }
$result = [pscustomobject][ordered]@{
    Line_follow_version = $words[1]
    State = $states[$words[2]]
    Stop_reason = $reasons[$words[3]]
    Black_mask_hex = '0x{0:X2}' -f $words[4]
    Selected_mask_hex = '0x{0:X2}' -f $words[24]
    Active_channels = $words[5]
    Position_left_negative = Signed32 $words[6]
    Steering_right_positive_cps = Signed32 $words[7]
    Base_cps = $words[8]
    Left_target_cps = $words[9]
    Right_target_cps = $words[10]
    Last_control_frame_age_ms = $words[11]
    Control_updates = $words[12]
    Uptime_ms = $words[13]
    Run_duration_ms = $words[14]
    Left_measured_cps = Signed32 $words[16]
    Right_measured_cps = Signed32 $words[17]
    Left_PWM_percent = Signed32 $words[18]
    Right_PWM_percent = Signed32 $words[19]
    Speed_fault = $faults[$words[20]]
    Start_rejections = $words[21]
    User_stops = $words[22]
    Automatic_stops = $words[23]
    Ignored_extra_group_frames = $words[25]
    Uncertain_frames = $words[26]
    Pending_reason = $reasons[$words[27]]
    Pending_ms = $words[28]
    Recovered_transients = $words[29]
    Last_uncertain_mask_hex = '0x{0:X2}' -f $words[30]
    Last_uncertain_reason = $reasons[$words[31]]
}
if ($AsJson) { $result | ConvertTo-Json } else {
    $result | Format-List
    Write-Host 'S1 = left; S8 = right. Start on the transverse stripe. Single click runs/stops; double click sets speed.'
    Write-Host 'Stop diagnostics retain the last targets. State/PWM indicate whether motors are running.'
}
