param(
    [string]$ProjectRoot=(Split-Path -Parent $PSScriptRoot),
    [string]$DumpFile,
    [switch]$AsJson
)
$ErrorActionPreference='Stop'
if ($DumpFile) { $lines=Get-Content -LiteralPath $DumpFile } else {
    $map=Get-Content -LiteralPath (Join-Path $ProjectRoot 'build\Debug\C10B_Test.map') -Raw
    $symbol=[regex]::Match($map,'(?m)^\s+0x([0-9a-fA-F]+)\s+oled_diag\s*$')
    if (-not $symbol.Success) { throw 'No oled_diag symbol. Build and flash OLED driver V2 first.' }
    $address='0x'+$symbol.Groups[1].Value
    $programmer=Join-Path $env:LOCALAPPDATA 'stm32cube\bundles\programmer\2.23.0\bin\STM32_Programmer_CLI.exe'
    if (-not (Test-Path -LiteralPath $programmer)) { throw "Programmer missing: $programmer" }
    $lines=& $programmer -c port=SWD mode=HOTPLUG -r32 $address 0x40 2>&1
    if ($LASTEXITCODE -ne 0) { throw "ST-LINK read failed: $($lines -join ' ')" }
}
$words=[Collections.Generic.List[uint32]]::new()
foreach($line in $lines) {
    $m=[regex]::Match([string]$line,'^\s*0x[0-9a-fA-F]+\s*:\s*((?:[0-9a-fA-F]{8}(?:\s+|$))+)')
    if($m.Success) { foreach($word in ($m.Groups[1].Value.Trim() -split '\s+')) { $words.Add([Convert]::ToUInt32($word,16)) } }
}
if($words.Count -ne 16) { throw "Incomplete OLED record: $($words.Count)/16 words." }
if($words[0] -ne 0x4F4C4432 -or $words[1] -ne 2) {
    throw ('OLED record mismatch: magic=0x{0:X8}, version={1}; expected 0x4F4C4432/version 2. Build and flash matching Debug ELF.' -f $words[0],$words[1])
}
$stages=@('PowerSettle','ResetLow','ResetWait','InitCommands','FirstFrame','StaticTest','Ready')
if($words[2] -ge $stages.Count) { throw 'Invalid OLED stage.' }
$dc=($words[8] -shr 12) -band 15
$rst=($words[8] -shr 16) -band 15
$sda=($words[8] -shr 20) -band 15
$scl=($words[10] -shr 24) -band 15
$trace=($words[12] -band 0x20) -ne 0
$lse=($words[13] -band 1) -ne 0
$r=[pscustomobject][ordered]@{
    OLED_driver_version=$words[1]
    Stage=$stages[$words[2]]
    Uptime_ms=$words[3]
    Frames_sent=$words[4]
    Bytes_sent=$words[5]
    Max_update_us=$words[6]
    Transfer_pending=[bool]$words[7]
    GPIO_configuration_matches=($dc -eq 2 -and $rst -eq 2 -and $sda -eq 2 -and $scl -eq 2 -and -not $trace -and -not $lse)
    PB3_DC_mode='0x{0:X}' -f $dc
    PB4_RST_mode='0x{0:X}' -f $rst
    PB5_SDA_mode='0x{0:X}' -f $sda
    PC14_SCL_mode='0x{0:X}' -f $scl
    Reset_output_high=($words[9] -band 0x10) -ne 0
    SWO_trace_pins_enabled=$trace
    Trace_was_enabled_at_init=($words[14] -band 0x20) -ne 0
    LSE_enabled=$lse
    Transfer_offset=$words[15]
}
if(-not $DumpFile) {
    $dir=Join-Path $ProjectRoot 'logs'
    New-Item -ItemType Directory -Force -Path $dir | Out-Null
    $path=Join-Path $dir ('oled-'+(Get-Date -Format 'yyyyMMdd-HHmmss-fff')+'.json')
    $r | ConvertTo-Json | Set-Content -LiteralPath $path -Encoding UTF8
    if(-not $AsJson) { Write-Host "Saved: $path" }
}
if($AsJson) { $r | ConvertTo-Json } else {
    $r | Format-List
    Write-Host 'Read-only snapshots. Frames_sent/Ready do not confirm a detected or working screen.'
}
