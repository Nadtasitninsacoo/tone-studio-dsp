<#
  Tone Studio Engine launcher.

  What the Desktop icon runs. It exists so nobody has to open a terminal, `cd` four folders
  deep, or know what an audio device is called:

    1. Finds the engine and the bridge next to itself (installed) or in the repo (development).
    2. Asks once which input and output to use, as a numbered menu built from
       `tone-studio-app --list-devices`, with the likely interface pre-selected, and remembers
       the answer in %APPDATA%\ToneStudioEngine\devices.json.
    3. Waits for that device if it is not plugged in yet, instead of failing — the engine itself
       refuses a name that matches nothing, which is right for the engine and wrong for an icon.
    4. Starts the bridge (minimised), then the engine in this window, and restarts the engine if
       it stops on its own. Closing this window stops both.

  -Reconfigure  forget the saved devices and ask again (the "เปลี่ยนอุปกรณ์เสียง" shortcut).
#>
param([switch]$Reconfigure)

# Continue, not Stop: under Windows PowerShell 5.1 a native program writing to stderr becomes an
# error record, and Stop would end the launcher on the engine's first warning line.
$ErrorActionPreference = 'Continue'
try { [Console]::OutputEncoding = [Text.Encoding]::UTF8 } catch { }
$Host.UI.RawUI.WindowTitle = 'Tone Studio Engine'

$here = Split-Path -Parent $MyInvocation.MyCommand.Path

function Say([string]$text, [string]$color = 'Gray') { Write-Host $text -ForegroundColor $color }

# ---------------------------------------------------------------------------------------------
# Where things are. Installed layout first, then the repo layout this script also lives in.
# ---------------------------------------------------------------------------------------------
function Find-Engine {
  $candidates = @(
    (Join-Path $here 'tone-studio-app.exe'),
    (Join-Path $here '..\build\app\tone-studio-app_artefacts\Release\tone-studio-app.exe')
  )
  foreach ($path in $candidates) { if (Test-Path $path) { return (Resolve-Path $path).Path } }
  return $null
}

function Find-Bridge {
  $exe = Join-Path $here 'tone-studio-bridge.exe'
  if (Test-Path $exe) { return @{ File = (Resolve-Path $exe).Path; Args = @() } }
  foreach ($js in @((Join-Path $here 'bridge.js'), (Join-Path $here '..\bridge.js'))) {
    if (Test-Path $js) {
      $node = Get-Command node -ErrorAction SilentlyContinue
      if ($node) { return @{ File = $node.Source; Args = @('"' + (Resolve-Path $js).Path + '"') } }
    }
  }
  return $null
}

$engine = Find-Engine
$bridge = Find-Bridge

Say ''
Say '  ===========================================' 'DarkYellow'
Say '     TONE STUDIO ENGINE' 'Yellow'
Say '  ===========================================' 'DarkYellow'
Say ''

if (-not $engine) {
  Say '  ไม่พบ tone-studio-app.exe — ติดตั้งใหม่ หรือ build เอนจินก่อน' 'Red'
  Read-Host '  กด Enter เพื่อปิด'
  exit 1
}
if (-not $bridge) {
  Say '  ไม่พบบริดจ์ (tone-studio-bridge.exe หรือ bridge.js + Node.js)' 'Red'
  Read-Host '  กด Enter เพื่อปิด'
  exit 1
}

# ---------------------------------------------------------------------------------------------
# Devices
# ---------------------------------------------------------------------------------------------
function Get-Devices {
  # Only the first section, [Windows Audio]: the engine opens that type when --device-type is
  # not given, and the other sections list the same hardware under driver-specific names.
  $lines = & $engine --list-devices 2>$null
  $inputs = @(); $outputs = @(); $section = 0
  foreach ($line in $lines) {
    if ($line -match '^\[(.+)\]') { $section += 1; continue }
    if ($section -ne 1) { continue }
    if ($line -match '--input\s+"(.+)"') { $inputs += $Matches[1] }
    elseif ($line -match '--output\s+"(.+)"') { $outputs += $Matches[1] }
  }
  return @{ Inputs = $inputs; Outputs = $outputs }
}

# A name that looks like an audio interface rather than a laptop's own sound chip.
function Test-Interface([string]$name) {
  if ($name -match 'Synaptics|Realtek|Intel|Conexant|High Definition|Default|Primary') { return $false }
  return $name -match 'USB|Focusrite|Scarlett|Behringer|X-USB|XR1|Yamaha|Steinberg|Audient|MOTU|RME|PreSonus|Zoom|Roland|Tascam|Interface'
}

function Select-Device([string]$title, [string[]]$names) {
  if ($names.Count -eq 0) { return $null }
  $recommended = 0
  for ($i = 0; $i -lt $names.Count; $i++) { if (Test-Interface $names[$i]) { $recommended = $i; break } }
  Say "  $title" 'Cyan'
  for ($i = 0; $i -lt $names.Count; $i++) {
    $mark = if ($i -eq $recommended) { '   <- แนะนำ' } else { '' }
    Say ("    {0}. {1}{2}" -f ($i + 1), $names[$i], $mark) $(if ($i -eq $recommended) { 'White' } else { 'Gray' })
  }
  while ($true) {
    $answer = Read-Host ("  พิมพ์เลข แล้วกด Enter [{0}]" -f ($recommended + 1))
    if ([string]::IsNullOrWhiteSpace($answer)) { return $names[$recommended] }
    $n = 0
    if ([int]::TryParse($answer.Trim(), [ref]$n) -and $n -ge 1 -and $n -le $names.Count) { return $names[$n - 1] }
    Say '  เลขไม่ถูกต้อง ลองใหม่' 'Red'
  }
}

$configDir = Join-Path $env:APPDATA 'ToneStudioEngine'
$configFile = Join-Path $configDir 'devices.json'

function Read-Config {
  if ($Reconfigure -or -not (Test-Path $configFile)) { return $null }
  try { return Get-Content $configFile -Raw -Encoding UTF8 | ConvertFrom-Json } catch { return $null }
}

function Save-Config($inputName, $outputName) {
  New-Item -ItemType Directory -Force -Path $configDir | Out-Null
  @{ input = $inputName; output = $outputName } | ConvertTo-Json | Set-Content -Path $configFile -Encoding UTF8
}

function Choose-Devices {
  $devices = Get-Devices
  Say '  เลือกอุปกรณ์เสียง (ครั้งเดียว — ระบบจะจำไว้)' 'Yellow'
  Say ''
  $in = Select-Device 'อินพุต · เสียงเข้าหาเอนจิน:' $devices.Inputs
  Say ''
  $out = Select-Device 'เอาต์พุต · เสียงออกลำโพง:' $devices.Outputs
  Say ''
  if (-not $in -or -not $out) {
    Say '  ไม่พบอุปกรณ์เสียงในเครื่อง — เสียบ interface แล้วเปิดใหม่' 'Red'
    Read-Host '  กด Enter เพื่อปิด'
    exit 1
  }
  Save-Config $in $out
  Say '  บันทึกแล้ว — ครั้งต่อไปเปิดแล้วทำงานทันที' 'Green'
  Say ''
  return @{ input = $in; output = $out }
}

$config = Read-Config
if (-not $config) { $config = Choose-Devices }

# Wait for the saved devices rather than failing: an icon clicked before the USB cable is in
# should start the moment it is, not print an error.
function Wait-ForDevices($config) {
  $warned = $false
  while ($true) {
    $devices = Get-Devices
    $hasIn = $devices.Inputs -contains $config.input
    $hasOut = $devices.Outputs -contains $config.output
    if ($hasIn -and $hasOut) { return $config }
    if (-not $warned) {
      if (-not $hasIn) { Say "  ไม่พบอินพุต  `"$($config.input)`"" 'DarkYellow' }
      if (-not $hasOut) { Say "  ไม่พบเอาต์พุต `"$($config.output)`"" 'DarkYellow' }
      Say '  เสียบอุปกรณ์แล้วรอสักครู่... (กด C เพื่อเลือกอุปกรณ์ใหม่)' 'DarkYellow'
      $warned = $true
    }
    for ($t = 0; $t -lt 20; $t++) {
      if ([Console]::KeyAvailable) {
        $key = [Console]::ReadKey($true)
        if ($key.Key -eq 'C') { Say ''; return (Choose-Devices) }
      }
      Start-Sleep -Milliseconds 100
    }
  }
}

# ---------------------------------------------------------------------------------------------
# Run
# ---------------------------------------------------------------------------------------------
# A previous run left running (its window closed with the X) still holds port 8080 and the audio
# device; starting beside it would fail on both. Clicking the icon means "I want it on", so the
# old pair is stopped and replaced.
$stale = Get-Process -Name 'tone-studio-app', 'tone-studio-bridge' -ErrorAction SilentlyContinue
if ($stale) {
  Say '  ปิดเอนจิน/บริดจ์ตัวเก่าที่ยังค้างอยู่...' 'DarkGray'
  $stale | Stop-Process -Force -ErrorAction SilentlyContinue
  Start-Sleep -Milliseconds 500
}

$bridgeProcess = $null
try {
  $startArgs = @{ FilePath = $bridge.File; WindowStyle = 'Minimized'; PassThru = $true }
  if ($bridge.Args.Count -gt 0) { $startArgs.ArgumentList = $bridge.Args }
  $bridgeProcess = Start-Process @startArgs
  Say '  บริดจ์ทำงานแล้ว · ws://127.0.0.1:8080' 'Green'

  while ($true) {
    $config = Wait-ForDevices $config
    Say ''
    Say "  อินพุต   : $($config.input)" 'White'
    Say "  เอาต์พุต : $($config.output)" 'White'
    Say '  เอนจินกำลังทำงาน — ย่อหน้าต่างนี้ได้ แต่อย่าปิด (ปิด = หยุดเอนจิน)' 'Green'
    Say ''
    & $engine --input $config.input --output $config.output
    $code = $LASTEXITCODE
    Say ''
    Say "  เอนจินหยุด (code $code) — เปิดใหม่ใน 3 วินาที · กด Ctrl+C เพื่อออก" 'DarkYellow'
    Start-Sleep -Seconds 3
  }
}
finally {
  if ($bridgeProcess -and -not $bridgeProcess.HasExited) { Stop-Process -Id $bridgeProcess.Id -Force -ErrorAction SilentlyContinue }
}
