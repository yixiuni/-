param(
    [string]$ProjectRoot = (Split-Path -Parent $PSScriptRoot),
    [string]$ProgrammerPath,
    [string]$DumpFile,
    [switch]$AsJson
)
$ErrorActionPreference='Stop'
function Parse-Words($Lines) {
    $result=[Collections.Generic.List[uint32]]::new()
    foreach ($line in $Lines) {
        $match=[regex]::Match([string]$line,'^\s*0x[0-9a-fA-F]+\s*:\s*((?:[0-9a-fA-F]{8}(?:\s+|$))+)')
        if ($match.Success) {
            foreach ($hex in ($match.Groups[1].Value.Trim() -split '\s+')) { $result.Add([Convert]::ToUInt32($hex,16)) }
        }
    }
    return ,$result.ToArray()
}
function Read-Words([string]$Address,[string]$Size) {
    $lines=& $ProgrammerPath -c port=SWD mode=HOTPLUG -r32 $Address $Size 2>&1
    if ($LASTEXITCODE -ne 0) { throw "ST-LINK read failed: $($lines -join ' ')" }
    return ,(Parse-Words $lines)
}
if ($DumpFile) { $words=Parse-Words (Get-Content -LiteralPath $DumpFile) }
else {
    if (-not $ProgrammerPath) { $ProgrammerPath=Join-Path $env:LOCALAPPDATA 'stm32cube\bundles\programmer\2.23.0\bin\STM32_Programmer_CLI.exe' }
    $map=Get-Content -LiteralPath (Join-Path $ProjectRoot 'build\Debug\C10B_Test.map') -Raw
    $symbol=[regex]::Match($map,'(?m)^\s+0x([0-9a-fA-F]+)\s+line_follow_trace\s*$')
    if (-not $symbol.Success) { throw 'Current Debug map has no control trace. Build/flash the matching recorder firmware.' }
    $address='0x'+$symbol.Groups[1].Value
    $before=Read-Words $address '0x40'
    if ($before.Count -ne 16 -or $before[0] -ne 0x54524331 -or $before[1] -ne 1) { throw 'Trace header/map mismatch.' }
    if ($before[9] -ne 0 -or ($before[10] -band 1)) { throw 'Trace is still recording. Stop with USER, then read again.' }
    $words=Read-Words $address '0x4040'
    $after=Read-Words $address '0x40'
    if ($after.Count -ne 16 -or $after[9] -ne 0 -or $after[10] -ne $before[10] -or
        $words.Count -lt 16 -or $words[10] -ne $before[10] -or $after[7] -ne $before[7]) {
        throw 'A new run started while reading; stop and retry.'
    }
}
if ($words.Count -ne 4112 -or $words[0] -ne 0x54524331 -or $words[1] -ne 1 -or
    $words[2] -ne 32 -or $words[3] -ne 512 -or $words[4] -ge 512 -or
    $words[5] -gt 512 -or $words[6] -lt $words[5] -or $words[9] -ne 0 -or ($words[10] -band 1)) {
    throw 'Invalid, active, or incomplete trace record.'
}
$raw=[Collections.Generic.List[byte]]::new()
foreach ($word in $words) { $raw.AddRange([BitConverter]::GetBytes([uint32]$word)) }
$bytes=$raw.ToArray()
$frames=@(for ($i=0; $i -lt $words[5]; $i++) {
    $index=($words[4]+512-$words[5]+$i)%512
    $offset=64+32*$index
    [pscustomobject][ordered]@{
        Time_ms=[BitConverter]::ToUInt32($bytes,$offset)
        Position=[BitConverter]::ToInt16($bytes,$offset+4)
        Steering_cps=[BitConverter]::ToInt16($bytes,$offset+6)
        Left_target_cps=[BitConverter]::ToUInt16($bytes,$offset+8)
        Right_target_cps=[BitConverter]::ToUInt16($bytes,$offset+10)
        Left_actual_cps=[BitConverter]::ToInt16($bytes,$offset+12)
        Right_actual_cps=[BitConverter]::ToInt16($bytes,$offset+14)
        Left_PWM=$bytes[$offset+16]; Right_PWM=$bytes[$offset+17]
        Mask=('0x{0:X2}' -f $bytes[$offset+18]); Selected_mask=('0x{0:X2}' -f $bytes[$offset+19])
        Frame_age_ms=[BitConverter]::ToUInt16($bytes,$offset+20)
        State=$bytes[$offset+22]; Stop_reason=$bytes[$offset+23]
        S4_strength=[BitConverter]::ToUInt16($bytes,$offset+24)
        S5_strength=[BitConverter]::ToUInt16($bytes,$offset+26)
        Filtered_position_rate=[BitConverter]::ToInt16($bytes,$offset+28)
        Pending_reason=$bytes[$offset+30]; Speed_fault=$bytes[$offset+31]
    }
})
$summary=[pscustomobject][ordered]@{
    Trace_version=$words[1]; Saved_samples=$words[5]; Total_samples=$words[6]
    Target_cps=$words[13]; Stop_reason=$words[11]; Nominal_sample_period_ms=$words[12]
    First_time_ms=$(if ($frames.Count) { $frames[0].Time_ms } else { $null })
    Last_time_ms=$(if ($frames.Count) { $frames[-1].Time_ms } else { $null })
}
$result=[pscustomobject][ordered]@{Summary=$summary;Frames=$frames}
if (-not $DumpFile) {
    $logs=Join-Path $ProjectRoot 'logs'
    New-Item -ItemType Directory -Path $logs -Force | Out-Null
    $path=Join-Path $logs ('control-trace-'+(Get-Date -Format 'yyyyMMdd-HHmmss-fff')+'.json')
    if (Test-Path -LiteralPath $path) { throw "Log already exists: $path" }
    $result | ConvertTo-Json -Depth 5 | Set-Content -LiteralPath $path -Encoding UTF8
    if (-not $AsJson) { Write-Host "Saved: $path" }
}
if ($AsJson) { $result | ConvertTo-Json -Depth 5 } else { $summary | Format-List }
