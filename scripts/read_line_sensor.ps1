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
    $symbol = [regex]::Match($mapText, '(?m)^\s+0x([0-9a-fA-F]+)\s+line_sensor\s*$')
    if (-not $symbol.Success) {
        throw 'The map has no line_sensor. Rebuild the updated line-sensor firmware.'
    }
    $address = '0x' + $symbol.Groups[1].Value
    $programmer = Join-Path $env:LOCALAPPDATA 'stm32cube\bundles\programmer\2.23.0\bin\STM32_Programmer_CLI.exe'
    if (-not (Test-Path -LiteralPath $programmer)) {
        throw "STM32CubeProgrammer not found: $programmer"
    }
    # Hot Plug only: no download, reset, start or halt command.
    $dumpLines = & $programmer -c port=SWD mode=HOTPLUG -r32 $address 0x1D0 2>&1
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

if ($words.Count -ne 116 -or $words[0] -ne 0x4C4E4631 -or $words[1] -ne 5) {
    throw 'Expected V5 analog/calibration firmware. Build and flash the new Debug ELF first.'
}
$states = @('BootListening', 'NoAnalogFrames', 'ReceivingAnalog', 'Stale', 'UartErrorsOnly')
$calStates = @('NotLearned', 'WhiteReference', 'Sweeping', 'Ready', 'Failed')
$errors = @('None', 'InsufficientFreshFrames', 'WhiteReferenceUnstable', 'InsufficientBlackWhiteContrast', 'FlashSaveFailed_RAMOnly', 'Cancelled')
if ($words[2] -ge $states.Count -or $words[38] -ge $calStates.Count -or $words[39] -ge $errors.Count) {
    throw 'Invalid state values. Ensure the ELF used to flash matches the current linker map.'
}
$fresh = $words[32] -gt 0 -and $words[37] -le 500
$active = $words[38] -eq 1 -or $words[38] -eq 2
$usable = $fresh -and $words[40] -eq 1 -and -not $active
$bits = for ($i = 0; $i -lt 8; $i++) { [string](($words[3] -shr $i) -band 1) }
$badChannels = for ($i = 0; $i -lt 8; $i++) { if (($words[44] -shr $i) -band 1) { 'S' + ($i + 1) } }
$rows = for ($i = 0; $i -lt 8; $i++) {
    [pscustomobject][ordered]@{
        Channel = 'S' + ($i + 1)
        Raw = $words[48 + $i]
        Learned_low = $words[56 + $i]
        Learned_high = $words[64 + $i]
        Threshold = $words[72 + $i]
        White_reference = $words[80 + $i]
        Latest_white_median = $words[100 + $i]
        Latest_white_spread = $words[108 + $i]
        Black_strength_0_to_1000 = $(if ($usable) { $words[88 + $i] } else { $null })
        Black = $(if ($usable) { ($words[3] -shr $i) -band 1 } else { $null })
    }
}
$result = [pscustomobject][ordered]@{
    Firmware_diagnostic_version = $words[1]
    State = $states[$words[2]]
    Fresh_analog_frame = $fresh
    Calibration = $calStates[$words[38]]
    Calibration_error = $errors[$words[39]]
    Calibration_valid = [bool]$words[40]
    Saved_to_flash = [bool]$words[41]
    Calibration_remaining_ms = $words[46]
    Calibration_samples = $words[43]
    White_reference_samples = $words[96]
    Learning_sample_period_ms = $words[97]
    Sweep_coverage_ms = $words[98]
    Failed_channels = ($badChannels -join ' ')
    Black_mask_usable = $usable
    Black_mask_hex = $(if ($usable) { '0x{0:X2}' -f $words[3] } else { 'Unavailable' })
    S1_to_S8_black_is_1 = $(if ($usable) { $bits -join ' ' } else { 'Unavailable' })
    Valid_analog_frames = $words[32]
    Average_RX_frames_per_second = $(if ($words[15] -gt 0) { [math]::Round(1000.0 * $words[32] / $words[15], 1) } else { 0 })
    RX_bytes = $words[4]
    TX_read_02_commands = $(if ($words[6] -gt 0) { $words[6] - 1 } else { 0 })
    Response_timeouts = $words[7]
    Frame_resync_discarded_bytes = $words[33]
    Checksum_errors = $words[34]
    Partial_frame_timeouts = $words[35]
    UART_errors = $words[8]
    Frame_age_ms = $words[37]
    Uptime_ms = $words[15]
    Channels = @($rows)
}
if ($AsJson) { $result | ConvertTo-Json -Depth 4 } else {
    $result | Select-Object * -ExcludeProperty Channels | Format-List
    $rows | Select-Object Channel, Raw, Learned_low, Learned_high, Threshold, Black_strength_0_to_1000, Black | Format-Table -AutoSize
    if ($words[38] -eq 4) { $rows | Select-Object Channel, Latest_white_median, Latest_white_spread | Format-Table -AutoSize }
    Write-Host 'Park on white at normal height. Hold USER >=6 seconds, then release; remain still on white for 1 second until fast flashing.'
    Write-Host 'During 10 seconds of fast flashing, sweep all eight probes over black and white several times.'
    Write-Host 'Steady LED1 = Ready; slow flashing = Failed or save error. KEY on sensor is not needed.'
    Write-Host 'Learning keeps motors stopped. Single click starts/stops at START stripe; double click sets speed; hold 2-6s/release sets laps.'
    Write-Host 'Software black bits are separate from the module LEDs/digital thresholds. S1..S8 order is not vehicle orientation.'
}
