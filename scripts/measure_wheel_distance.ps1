param(
    [ValidateSet('Read','Begin','Finish')][string]$Action = 'Read',
    [string]$ProjectRoot = 'E:\stm32\C10B_Test',
    [ValidateRange(300,10000)][int]$DistanceMm = 1000,
    [string]$BaselinePath,
    [string]$ProgrammerPath,
    [string]$DumpFile
)
$ErrorActionPreference = 'Stop'

function Decode-Encoder($Lines) {
    $bytes = [System.Collections.Generic.List[byte]]::new()
    foreach ($line in $Lines) {
        $m = [regex]::Match([string]$line, '^\s*0x[0-9a-fA-F]+\s*:\s*((?:[0-9a-fA-F]{8}(?:\s+|$))+)')
        if ($m.Success) {
            foreach ($word in ($m.Groups[1].Value.Trim() -split '\s+')) {
                $bytes.AddRange([BitConverter]::GetBytes([Convert]::ToUInt32($word,16)))
            }
        }
    }
    if ($bytes.Count -ne 48) { throw "Expected 48 encoder bytes, got $($bytes.Count)." }
    $b = $bytes.ToArray()
    # STM32 ARM EABI: 64-bit totals at offsets 16 and 24; sizeof Encoder_Data=48.
    [pscustomobject]@{
        Left_total_counts = [BitConverter]::ToInt64($b,16)
        Right_total_counts = [BitConverter]::ToInt64($b,24)
        Left_cps = [BitConverter]::ToSingle($b,32)
        Right_cps = [BitConverter]::ToSingle($b,36)
        Sample_count = [BitConverter]::ToUInt32($b,44)
    }
}

# Offline decoding only. Cannot create calibration results from a dump.
if ($DumpFile) {
    if ($Action -ne 'Read') { throw 'DumpFile is supported only with Action Read.' }
    Decode-Encoder (Get-Content -LiteralPath $DumpFile) | ConvertTo-Json
    return
}
if (-not $BaselinePath) { $BaselinePath = Join-Path $ProjectRoot 'logs\wheel-distance-baseline.json' }
$elf = Join-Path $ProjectRoot 'build\Debug\C10B_Test.elf'
$mapPath = Join-Path $ProjectRoot 'build\Debug\C10B_Test.map'
$mapText = Get-Content -LiteralPath $mapPath -Raw
$symbol = [regex]::Match($mapText, '(?m)^\s+0x([0-9a-fA-F]+)\s+encoder_data\s*$')
if (-not $symbol.Success) { throw 'encoder_data is missing from the Debug map.' }
$address = '0x' + $symbol.Groups[1].Value
function File-SHA256([string]$Path) {
    $sha = [Security.Cryptography.SHA256]::Create()
    $stream = [IO.File]::OpenRead($Path)
    try { return [BitConverter]::ToString($sha.ComputeHash($stream)).Replace('-','') }
    finally { $stream.Dispose(); $sha.Dispose() }
}
$elfHash = File-SHA256 $elf
$mapHash = File-SHA256 $mapPath
if (-not $ProgrammerPath) {
    $ProgrammerPath = Join-Path $env:LOCALAPPDATA 'stm32cube\bundles\programmer\2.23.0\bin\STM32_Programmer_CLI.exe'
}
if (-not (Test-Path -LiteralPath $ProgrammerPath)) { throw "Programmer not found: $ProgrammerPath" }

function Read-ParkedStatus {
    $r = & (Join-Path $ProjectRoot 'scripts\read_line_follow.ps1') -ProjectRoot $ProjectRoot -AsJson | ConvertFrom-Json
    if ($r.Line_follow_version -notin @(2,3)) { throw 'This tool requires line-follow V2 or task2 line-follow V3 firmware.' }
    if ($r.State -eq 'Following' -or $r.Left_PWM_percent -ne 0 -or $r.Right_PWM_percent -ne 0) {
        throw 'Stop the car with USER before measuring. This script never controls the motors.'
    }
    return $r
}
function Read-Encoder {
    $lines = & $ProgrammerPath -c port=SWD mode=HOTPLUG -r32 $address 0x30 2>&1
    if ($LASTEXITCODE -ne 0) { throw "ST-LINK read failed: $($lines -join ' ')" }
    Decode-Encoder $lines
}

$before = Read-ParkedStatus
$first = Read-Encoder
Start-Sleep -Milliseconds 150
$current = Read-Encoder
$status = Read-ParkedStatus
if ($first.Left_total_counts -ne $current.Left_total_counts -or
    $first.Right_total_counts -ne $current.Right_total_counts) {
    throw 'Wheels moved during the reading. Hold the car still and retry.'
}
if ($current.Sample_count -le $first.Sample_count -or $status.Uptime_ms -le $before.Uptime_ms) {
    throw 'Firmware is halted, reset, or counters wrapped during the reading. Retry with a fresh Begin.'
}
if ($Action -eq 'Read') { $current | Format-List; return }
$snapshot = [ordered]@{
    Format_version = 1
    Time = (Get-Date -Format o)
    Project_root = [IO.Path]::GetFullPath($ProjectRoot)
    ELF_SHA256 = $elfHash
    Map_SHA256 = $mapHash
    Uptime_ms = $status.Uptime_ms
    Control_updates = $status.Control_updates
    Run_duration_ms = $status.Run_duration_ms
    State = $status.State
    Start_rejections = $status.Start_rejections
    User_stops = $status.User_stops
    Automatic_stops = $status.Automatic_stops
    Encoder = $current
}
if ($Action -eq 'Begin') {
    New-Item -ItemType Directory -Force -Path (Split-Path -Parent ([IO.Path]::GetFullPath($BaselinePath))) | Out-Null
    $snapshot | ConvertTo-Json -Depth 4 | Set-Content -LiteralPath $BaselinePath -Encoding UTF8
    Write-Host "Baseline saved: $BaselinePath"
    Write-Host 'Roll the parked car straight forward over a measured distance (default 1000 mm), with both wheels on the floor.'
    Write-Host 'Keep power/ST-LINK connected. Do not press USER/RESET, flash, or start driving. Hold still, then run -Action Finish -DistanceMm <distance>.'
    return
}
$baseline = Get-Content -LiteralPath $BaselinePath -Raw | ConvertFrom-Json
if ($baseline.Format_version -ne 1 -or $baseline.ELF_SHA256 -ne $elfHash -or $baseline.Map_SHA256 -ne $mapHash -or
    $baseline.Project_root -ne $snapshot.Project_root) { throw 'Baseline does not match this build/project. Run Begin again.' }
if ($status.Uptime_ms -le $baseline.Uptime_ms -or $current.Sample_count -le $baseline.Encoder.Sample_count) {
    throw 'Reset/counter rollover detected. Run Begin again.'
}
foreach ($field in @('Control_updates','Run_duration_ms','State','Start_rejections','User_stops','Automatic_stops')) {
    if ($status.$field -ne $baseline.$field) { throw "Drive/button state changed ($field). Run Begin again without driving or pressing USER." }
}
$elapsed = ([datetime]$snapshot.Time - [datetime]$baseline.Time).TotalMilliseconds
$uptimeDelta = [double]$status.Uptime_ms - [double]$baseline.Uptime_ms
if ($elapsed -gt 600000 -or [math]::Abs($elapsed - $uptimeDelta) -gt 3000) {
    throw 'Measurement exceeded 10 minutes, or clock/reset/debug interruption detected. Run Begin again.'
}
$left = [long]$current.Left_total_counts - [long]$baseline.Encoder.Left_total_counts
$right = [long]$current.Right_total_counts - [long]$baseline.Encoder.Right_total_counts
if ($left -lt 1000 -or $right -lt 1000) { throw 'Insufficient forward counts on both wheels. Roll forward without lifting or slipping wheels; repeat Begin/Finish.' }
if ([math]::Abs($left - $right) / [math]::Max($left,$right) -gt 0.20) {
    throw 'Left/right counts differ by over 20%. Repeat a straight roll with both wheels on the floor.'
}
$result = [ordered]@{
    Time = $snapshot.Time
    Status = 'Measured_candidate_not_applied'
    Distance_mm = $DistanceMm
    Left_counts = $left
    Right_counts = $right
    Left_counts_per_mm = $left / [double]$DistanceMm
    Right_counts_per_mm = $right / [double]$DistanceMm
    Baseline = $baseline
    Finish = $snapshot
}
$output = Join-Path $ProjectRoot ('logs\wheel-distance-' + (Get-Date -Format 'yyyyMMdd-HHmmss-fff') + '.json')
$result | ConvertTo-Json -Depth 6 | Set-Content -LiteralPath $output -Encoding UTF8
[pscustomobject]$result | Select-Object Status,Distance_mm,Left_counts,Right_counts,Left_counts_per_mm,Right_counts_per_mm | Format-List
Write-Host "Saved: $output"
Write-Host 'Candidate only; no firmware or settings were changed. Repeat from a new Begin to check consistency.'
