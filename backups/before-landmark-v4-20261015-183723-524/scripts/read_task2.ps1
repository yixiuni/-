param(
    [string]$ProjectRoot = (Split-Path -Parent $PSScriptRoot),
    [string]$DumpFile,
    [switch]$AsJson,
    [switch]$Watch,
    [ValidateRange(5,600)][int]$DurationSeconds = 120,
    [string]$LogPath
)
$ErrorActionPreference = 'Stop'
if ($Watch) {
    if ($DumpFile -or $AsJson) { throw 'Watch cannot be combined with DumpFile/AsJson.' }
    if (-not $LogPath) { $LogPath=Join-Path $ProjectRoot ('logs\task2-'+(Get-Date -Format 'yyyyMMdd-HHmmss-fff')+'.jsonl') }
    New-Item -ItemType Directory -Force -Path (Split-Path -Parent ([IO.Path]::GetFullPath($LogPath))) | Out-Null
    if (Test-Path -LiteralPath $LogPath) { throw "Log exists: $LogPath" }
    Write-Host 'Read-only HOTPLUG. Start at the course origin on a clear line. Laps use encoder distance. Click=start/stop; 2 clicks=speed; hold 2-6s/release=laps.'
    Write-Host 'Hold >=6s/release=calibration. Watch ending does not stop motors; click USER to stop.'
    Write-Host "Log: $LogPath"
    $timer=[Diagnostics.Stopwatch]::StartNew()
    while ($timer.Elapsed.TotalSeconds -lt $DurationSeconds) {
        $stamp=Get-Date -Format 'HH:mm:ss.fff'
        try {
            $r=& $PSCommandPath -ProjectRoot $ProjectRoot -AsJson | ConvertFrom-Json
            [ordered]@{Time=$stamp;Snapshot=$r} | ConvertTo-Json -Depth 4 -Compress | Add-Content -LiteralPath $LogPath -Encoding UTF8
            Write-Host ('{0} {1} {2} lap={3}/{4} target={5}cps actual={6}cps distance={7}mm marker={8} armed={9}' -f
                $stamp,$r.State,$r.Stop_reason,$r.Completed_laps,$r.Target_laps,$r.Target_cps,$r.Actual_cps,$r.Distance_mm,$r.On_marker,$r.Lap_armed)
        } catch {
            $message=$_.Exception.Message
            [ordered]@{Time=$stamp;Read_error=$message} | ConvertTo-Json -Compress | Add-Content -LiteralPath $LogPath -Encoding UTF8
            Write-Host "$stamp READ_ERROR $message" -ForegroundColor Yellow
        }
        Start-Sleep -Milliseconds 500
    }
    Write-Host "Full record: $LogPath"
    return
}
if (-not $DumpFile) {
    $map=Get-Content -LiteralPath (Join-Path $ProjectRoot 'build\Debug\C10B_Test.map') -Raw
    $symbol=[regex]::Match($map,'(?m)^\s+0x([0-9a-fA-F]+)\s+task2_data\s*$')
    if (-not $symbol.Success) { throw 'Debug map has no task2_data. Build and flash the task2 firmware first.' }
    $address='0x'+$symbol.Groups[1].Value
    $programmer=Join-Path $env:LOCALAPPDATA 'stm32cube\bundles\programmer\2.23.0\bin\STM32_Programmer_CLI.exe'
    if (-not (Test-Path -LiteralPath $programmer)) { throw "Programmer missing: $programmer" }
}
$words=$null
for ($attempt=0;$attempt -lt 4;$attempt++) {
    if ($DumpFile) { $lines=Get-Content -LiteralPath $DumpFile } else {
        $lines=& $programmer -c port=SWD mode=HOTPLUG -r32 $address 0x80 2>&1
        if ($LASTEXITCODE -ne 0) { throw "ST-LINK read failed: $($lines -join ' ')" }
    }
    $words=[Collections.Generic.List[uint32]]::new()
    foreach ($line in $lines) {
        $m=[regex]::Match([string]$line,'^\s*0x[0-9a-fA-F]+\s*:\s*((?:[0-9a-fA-F]{8}(?:\s+|$))+)')
        if ($m.Success) { foreach ($word in ($m.Groups[1].Value.Trim() -split '\s+')) { $words.Add([Convert]::ToUInt32($word,16)) } }
    }
    if ($words.Count -ne 32) { throw "Incomplete record: $($words.Count)/32 words." }
    if ($words[0] -ne 0x54324331 -or $words[1] -notin @(1,2,3)) {
        throw ('Task2 firmware/map mismatch: magic=0x{0:X8}, version={1}; expected 0x54324331/version 1, 2 or 3. Rebuild and flash the matching Debug ELF.' -f $words[0],$words[1])
    }
    if ($words[2] -eq $words[31] -and ($words[2] -band 1) -eq 0) { break }
    if ($DumpFile -or $attempt -eq 3) { throw 'Snapshot changed while reading. Retry while keeping the car still.' }
    Start-Sleep -Milliseconds 30
}
$states=@('Parked','Running','Complete','Fault','StartRejected','Learning')
$reasons=@('None','USER','NotCalibrated','SensorStale','NoLine','AmbiguousLine','SpeedFault','Calibrating','LapsComplete','MarkerTimeout','NeedStartMarker','DistanceNotCalibrated')
if ($words[3] -ge $states.Count -or $words[4] -ge $reasons.Count) { throw 'Invalid task2 state.' }
function Signed([uint32]$v) { [BitConverter]::ToInt32([BitConverter]::GetBytes($v),0) }
$r=[pscustomobject][ordered]@{
    Task2_version=$words[1]
    State=$states[$words[3]]
    Stop_reason=$reasons[$words[4]]
    Speed_level=$words[5]+1
    Target_cps=$words[6]
    Selected_laps=$(if ($words[1] -eq 3) { $words[10] } else { $words[7] })
    Target_laps=$words[7]
    Completed_laps=$words[8]
    Lap_counting_method=$(if ($words[1] -ge 2) { 'EncoderDistance' } else { 'TransverseStripe' })
    Lap_armed=[bool]$words[9]
    On_marker=$(if ($words[1] -eq 3) { ($words[19] -band 0x7E) -eq 0x7E } else { [bool]$words[10] })
    Run_ms=$words[11]
    Mean_counts=Signed $words[12]
    Distance_mm=$(if ($words[21]) { Signed $words[13] } else { $null })
    Actual_cps=Signed $words[14]
    Speed_mm_s=$(if ($words[21]) { Signed $words[15] } else { $null })
    Speed_cm_s=$(if ($words[21]) { (Signed $words[15])/10.0 } else { $null })
    Left_counts=Signed $words[16]
    Right_counts=Signed $words[17]
    Sensor_calibrated=[bool]$words[18]
    Black_mask_hex='0x{0:X2}' -f $words[19]
    Frame_age_ms=$words[20]
    Distance_calibrated=[bool]$words[21]
    OLED_frames_sent=$words[22]
    OLED_max_update_us=$words[23]
    Key_hint=$words[24]
    Uptime_ms=$words[25]
    Left_cps=Signed $words[26]
    Right_cps=Signed $words[27]
    Lap_length_mm=$(if ($words[1] -ge 2) { $words[28] } else { $null })
    Target_distance_mm=$(if ($words[1] -ge 2) { $words[29] } else { $null })
    Selected_target_distance_mm=$(if ($words[1] -eq 3) { $words[10]*$words[28] } elseif ($words[1] -eq 2) { $words[29] } else { $null })
    Remaining_distance_mm=$(if ($words[1] -ge 2 -and $words[21]) { [math]::Max(0, [int64]$words[29]-(Signed $words[13])) } else { $null })
    Marker_entries=$(if ($words[1] -eq 1) { $words[28] } else { $null })
    Ignored_early_markers=$(if ($words[1] -eq 1) { $words[29] } else { $null })
    Last_lap_ms=$words[30]
}
if ($AsJson) { $r | ConvertTo-Json } else {
    $r | Format-List
    Write-Host 'OLED_frames_sent counts writes, not screen detection. Null physical units mean distance calibration is pending.'
}
