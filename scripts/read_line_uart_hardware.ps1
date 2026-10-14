param(
    [string]$DumpFile,
    [switch]$AsJson
)

$ErrorActionPreference = 'Stop'
# STM32F103 UART4 read-only diagnostics. Never read DR: it consumes data.
# SR is sampled alone. This does not clear RXNE or consume received bytes;
# the firmware still performs the normal SR/DR sequence in its IRQ handler.
$blocks = @(
    @{ Address = '0x40021004'; Size = '0x4' },  # RCC CFGR
    @{ Address = '0x40021018'; Size = '0x8' },  # APB2ENR, APB1ENR
    @{ Address = '0x40011004'; Size = '0x8' },  # GPIOC CRH, IDR
    @{ Address = '0x40004C08'; Size = '0x10' }, # UART4 BRR, CR1, CR2, CR3
    @{ Address = '0x40004C00'; Size = '0x4' },  # UART4 SR ONLY; no DR read
    @{ Address = '0xE000E104'; Size = '0x4' },  # NVIC ISER1, IRQ52 is bit20
    @{ Address = '0xE000E204'; Size = '0x4' }   # NVIC ISPR1
)
$memory = @{}
function Add-DumpLines($Lines) {
    foreach ($line in $Lines) {
        $match = [regex]::Match([string]$line, '^\s*0x([0-9a-fA-F]+)\s*:\s*((?:[0-9a-fA-F]{8}(?:\s+|$))+)')
        if (-not $match.Success) { continue }
        $address = [Convert]::ToUInt32($match.Groups[1].Value, 16)
        foreach ($word in ($match.Groups[2].Value.Trim() -split '\s+')) {
            $memory[('{0:X8}' -f $address)] = [Convert]::ToUInt32($word, 16)
            $address += 4
        }
    }
}
if ($DumpFile) {
    Add-DumpLines (Get-Content -LiteralPath $DumpFile)
} else {
    $programmer = Join-Path $env:LOCALAPPDATA 'stm32cube\bundles\programmer\2.23.0\bin\STM32_Programmer_CLI.exe'
    if (-not (Test-Path -LiteralPath $programmer)) { throw "STM32CubeProgrammer not found: $programmer" }
    foreach ($block in $blocks) {
        $lines = & $programmer -c port=SWD mode=HOTPLUG -r32 $block.Address $block.Size 2>&1
        if ($LASTEXITCODE -ne 0) {
            $lines | Out-Host
            throw 'ST-LINK read failed. End any active debugger session and retry.'
        }
        Add-DumpLines $lines
    }
}
function Get-Register([string]$Address) {
    if (-not $memory.ContainsKey($Address)) { throw "Missing register 0x$Address in programmer output." }
    return $memory[$Address]
}
$cfgr = Get-Register '40021004'
$apb2 = Get-Register '40021018'
$apb1 = Get-Register '4002101C'
$crh = Get-Register '40011004'
$idr = Get-Register '40011008'
$brr = Get-Register '40004C08'
$cr1 = Get-Register '40004C0C'
$cr2 = Get-Register '40004C10'
$cr3 = Get-Register '40004C14'
$sr = Get-Register '40004C00'
$iser1 = Get-Register 'E000E104'
$ispr1 = Get-Register 'E000E204'

# Board/project HSE is 8 MHz. This derives configured baud, not an electrical
# measurement of the oscillator or serial waveform.
$sysclk = $null
$source = ($cfgr -shr 2) -band 3
if ($source -eq 0 -or $source -eq 1) { $sysclk = 8000000 }
elseif ($source -eq 2) {
    $pllInput = if ($cfgr -band 0x10000) { 8000000 } else { 4000000 }
    if (($cfgr -band 0x10000) -and ($cfgr -band 0x20000)) { $pllInput /= 2 }
    $multiplier = [Math]::Min(16, (($cfgr -shr 18) -band 15) + 2)
    $sysclk = $pllInput * $multiplier
}
$ahbDiv = @(1,1,1,1,1,1,1,1,2,4,8,16,64,128,256,512)[($cfgr -shr 4) -band 15]
$apbDiv = @(1,1,1,1,2,4,8,16)[($cfgr -shr 8) -band 7]
$pclk = if ($null -ne $sysclk) { $sysclk / $ahbDiv / $apbDiv } else { $null }
$baud = if ($null -ne $pclk -and ($brr -band 0xffff) -gt 0) { [Math]::Round($pclk / ($brr -band 0xffff)) } else { $null }
$txMode = ($crh -shr 8) -band 15
$rxMode = ($crh -shr 12) -band 15
$gpioClock = ($apb2 -band 0x10) -ne 0
$uartClock = ($apb1 -band 0x80000) -ne 0
$uartFormat = ($cr1 -band 0x340c) -eq 0x200c -and ($cr2 -band 0x3000) -eq 0 -and ($cr3 -band 0x300) -eq 0
$result = [pscustomobject][ordered]@{
    Configuration_matches_expected = ($gpioClock -and $uartClock -and $txMode -eq 11 -and $rxMode -eq 4 -and $uartFormat -and $null -ne $baud -and [Math]::Abs($baud - 115200) -lt 1800)
    GPIOC_clock_enabled = $gpioClock
    UART4_clock_enabled = $uartClock
    UART4_enabled_TX_RX_8N1 = $uartFormat
    Configured_baud_assuming_8MHz_HSE = $baud
    PC10_TX_configuration = ('0x{0:X1} (expected 0xB)' -f $txMode)
    PC11_RX_configuration = ('0x{0:X1} (expected 0x4)' -f $rxMode)
    PC10_level = ($idr -shr 10) -band 1
    PC11_level = ($idr -shr 11) -band 1
    RCC_CFGR = ('0x{0:X8}' -f $cfgr)
    UART4_BRR = ('0x{0:X8}' -f $brr)
    UART4_CR1 = ('0x{0:X8}' -f $cr1)
    UART4_CR2 = ('0x{0:X8}' -f $cr2)
    UART4_CR3 = ('0x{0:X8}' -f $cr3)
    UART4_RXNE_interrupt_enabled = ($cr1 -band 0x20) -ne 0
    UART4_error_interrupt_enabled = ($cr3 -band 1) -ne 0
    NVIC_UART4_enabled = ($iser1 -band 0x100000) -ne 0
    NVIC_UART4_pending = ($ispr1 -band 0x100000) -ne 0
    UART4_RX_data_waiting = ($sr -band 0x20) -ne 0
    UART4_TX_register_empty = ($sr -band 0x80) -ne 0
    UART4_transmit_complete_now = ($sr -band 0x40) -ne 0
    UART4_SR = ('0x{0:X8}' -f $sr)
}
if ($AsJson) { $result | ConvertTo-Json } else {
    $result | Format-List
    Write-Host 'Pin levels are single snapshots. RX high alone does not prove the cable is connected.'
}
