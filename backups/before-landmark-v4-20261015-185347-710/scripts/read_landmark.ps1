param(
    [string]$ProjectRoot=(Split-Path -Parent $PSScriptRoot),
    [string]$DumpFile,
    [switch]$AsJson
)
$ErrorActionPreference='Stop'
if (-not $DumpFile) {
    $map=Get-Content -LiteralPath (Join-Path $ProjectRoot 'build\Debug\C10B_Test.map') -Raw
    $symbol=[regex]::Match($map,'(?m)^\s+0x([0-9a-fA-F]+)\s+landmark_diag\s*$')
    if (-not $symbol.Success) { throw 'Map does not contain landmark diagnostics. Build and flash the matching V4 firmware.' }
    $address='0x'+$symbol.Groups[1].Value
    $programmer=Join-Path $env:LOCALAPPDATA 'stm32cube\bundles\programmer\2.23.0\bin\STM32_Programmer_CLI.exe'
}
for ($attempt=0;$attempt -lt 4;$attempt++) {
    if ($DumpFile) { $lines=Get-Content -LiteralPath $DumpFile }
    else {
        $lines=& $programmer -c port=SWD mode=HOTPLUG -r32 $address 0xA0 2>&1
        if ($LASTEXITCODE -ne 0) { throw "ST-LINK read failed: $($lines -join ' ')" }
    }
    $words=[Collections.Generic.List[uint32]]::new()
    foreach ($line in $lines) {
        $m=[regex]::Match([string]$line,'^\s*0x[0-9a-fA-F]+\s*:\s*((?:[0-9a-fA-F]{8}(?:\s+|$))+)')
        if ($m.Success) { foreach ($hex in ($m.Groups[1].Value.Trim() -split '\s+')) { $words.Add([Convert]::ToUInt32($hex,16)) } }
    }
    if ($words.Count -ne 40 -or $words[0] -ne 0x4C415031 -or $words[1] -ne 1) { throw 'Landmark firmware/map mismatch or incomplete snapshot.' }
    if ($words[2] -eq $words[39] -and ($words[2] -band 1) -eq 0) { break }
    if ($DumpFile -or $attempt -eq 3) { throw 'Diagnostic snapshot changed; retry.' }
    Start-Sleep -Milliseconds 30
}
function Signed([uint32]$v) { [BitConverter]::ToInt32([BitConverter]::GetBytes($v),0) }
$phases=@('Idle','LeavingStartOrLandmark','Following','FinalApproach','Positioning','Complete','Fault')
$faults=@('None','LandmarkMissing','StripeTooWide','InvalidDistance','InvalidLandmarkInterval')
$position=@('Idle','Correcting','Settling','Complete','Fault')
if ($words[3] -ge $phases.Count -or $words[4] -ge $faults.Count -or $words[18] -ge $position.Count) { throw 'Invalid landmark state.' }
$result=[pscustomobject][ordered]@{
    Version=$words[1]; Phase=$phases[$words[3]]; Fault=$faults[$words[4]]
    Start_ready=[bool]$words[7]; Landmark_width_mm=$words[8]
    Confirmed_returns=$words[5]; Rejected_early_landmarks=$words[6]
    Completed_laps=$words[28]; Target_laps=$words[29]; Target_cps=$words[27]
    Raw_encoder_distance_mm=Signed $words[9]; Corrected_distance_mm=Signed $words[10]
    Last_landmark_center_raw_mm=Signed $words[11]; Previous_center_raw_mm=Signed $words[12]
    Last_landmark_correction_mm=Signed $words[13]; Stop_target_raw_mm=Signed $words[14]
    Landmark_armed=[bool]$words[15]; Waiting_to_cross_center=[bool]$words[16]
    Candidate_frames=$words[17]; Position_state=$position[$words[18]]
    Left_position_error_mm=(Signed $words[19])/1000.0
    Right_position_error_mm=(Signed $words[20])/1000.0
    Parking_left_target_cps=Signed $words[21]; Parking_right_target_cps=Signed $words[22]
    Position_tolerance_mm=$words[23]/1000.0; Cruise_limit_cps=$words[26]
    Left_actual_cps=Signed $words[34]; Right_actual_cps=Signed $words[35]
    Black_mask_hex='0x{0:X2}' -f $words[36]; Frame_age_ms=$words[37]
    Run_ms=$words[30]; Last_lap_ms=$words[31]; Uptime_ms=$words[24]
}
if ($AsJson) { $result | ConvertTo-Json } else { $result | Format-List }
