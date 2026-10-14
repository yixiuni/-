param(
    [string]$ProjectRoot = (Split-Path -Parent $PSScriptRoot),
    [string]$DumpFile,
    [switch]$AsJson
)

$ErrorActionPreference = 'Stop'

if ($DumpFile) {
    $dumpLines = Get-Content -LiteralPath $DumpFile
} else {
    $mapPath = Join-Path $ProjectRoot 'build\Debug\C10B_Test.map'
    if (-not (Test-Path -LiteralPath $mapPath)) {
        throw 'Build Debug successfully first: the linker map was not found.'
    }
    $mapText = Get-Content -LiteralPath $mapPath -Raw
    $symbol = [regex]::Match($mapText, '(?m)^\s+0x([0-9a-fA-F]+)\s+motor_test_result\s*$')
    if (-not $symbol.Success) {
        throw 'The map has no motor_test_result. Rebuild the updated speed-control firmware.'
    }
    $address = '0x' + $symbol.Groups[1].Value
    $programmer = Join-Path $env:LOCALAPPDATA 'stm32cube\bundles\programmer\2.23.0\bin\STM32_Programmer_CLI.exe'
    if (-not (Test-Path -LiteralPath $programmer)) {
        throw "STM32CubeProgrammer not found: $programmer"
    }
    # Hot Plug only: no download, reset, start or halt command.
    $dumpLines = & $programmer -c port=SWD mode=HOTPLUG -r32 $address 0x40 2>&1
    if ($LASTEXITCODE -ne 0) {
        $dumpLines | Out-Host
        throw 'Could not read ST-LINK. End any active debugging session and retry.'
    }
}

$words = [System.Collections.Generic.List[uint32]]::new()
foreach ($line in $dumpLines) {
    $match = [regex]::Match([string]$line, '^\s*0x[0-9a-fA-F]+\s*:\s*((?:[0-9a-fA-F]{8}(?:\s+|$))+)')
    if ($match.Success) {
        foreach ($word in ($match.Groups[1].Value.Trim() -split '\s+')) {
            $words.Add([Convert]::ToUInt32($word, 16))
        }
    }
}
if ($words.Count -ne 16 -or $words[0] -ne 0x43313042 -or $words[1] -ne 3) {
    throw 'The diagnostic record does not match this firmware. Build and flash the new Debug ELF; do not use the old memory address.'
}
function Get-SignedWord([int]$Index) {
    return [BitConverter]::ToInt32([BitConverter]::GetBytes($words[$Index]), 0)
}

$states = @('WaitingForButton', 'Running', 'Stopped', 'Fault')
$faults = @('None', 'Left encoder has no feedback', 'Right encoder has no feedback',
            'Encoder direction reversed', 'Sampling timed out', 'Invalid target speed')
$stateLabel = if ($words[2] -lt $states.Count) { $states[$words[2]] } else { 'Unknown' }
$faultLabel = if ($words[3] -lt $faults.Count) { $faults[$words[3]] } else { 'Unknown' }
$result = [pscustomobject][ordered]@{
    State = $stateLabel
    Fault = $faultLabel
    Target_counts_per_second = $words[4]
    Duration_ms = $words[5]
    Left_total_counts = Get-SignedWord 6
    Right_total_counts = Get-SignedWord 7
    Left_mean_counts_per_second = Get-SignedWord 8
    Right_mean_counts_per_second = Get-SignedWord 9
    Left_last_second_counts_per_second = Get-SignedWord 10
    Right_last_second_counts_per_second = Get-SignedWord 11
    Left_last_PWM_percent = Get-SignedWord 12
    Right_last_PWM_percent = Get-SignedWord 13
    Last_second_duration_ms = $words[14]
    Samples = $words[15]
}
if ($AsJson) {
    $result | ConvertTo-Json
} else {
    $result | Format-List
    if ($words[2] -lt 2) { Write-Host 'Values are finalized after stopping. USER toggles start/stop; power-on does not start the car.' }
}
