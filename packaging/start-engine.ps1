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
    4. Starts the bridge (minimised), then the engine in this window, both inside a job object
       that dies with the window — closing it stops both, always. The engine is restarted only
       if it crashes, and a device that keeps failing stops the loop with a message.
    5. One launcher at a time: a second click brings this window forward.

  -Reconfigure  forget the saved devices and ask again (the "เปลี่ยนอุปกรณ์เสียง" shortcut).
#>
param([switch]$Reconfigure)

# Continue, not Stop: under Windows PowerShell 5.1 a native program writing to stderr becomes an
# error record, and Stop would end the launcher on the engine's first warning line.
$ErrorActionPreference = 'Continue'
try { [Console]::OutputEncoding = [Text.Encoding]::UTF8 } catch { }
$Host.UI.RawUI.WindowTitle = 'Tone Studio Engine'

$here = Split-Path -Parent $MyInvocation.MyCommand.Path

# One launcher at a time. A second click on the icon brings the running window forward instead
# of starting a second engine on the same device — two engines were what kept howling after the
# first install.
$launcherMutex = New-Object System.Threading.Mutex($false, 'Local\ToneStudioEngineLauncher')
$owned = $false
try { $owned = $launcherMutex.WaitOne(0) } catch [System.Threading.AbandonedMutexException] { $owned = $true }
$pidFile = Join-Path (Join-Path $env:APPDATA 'ToneStudioEngine') 'launcher.pid'

# "Change Audio Device" while the engine is running: close the running launcher and carry on
# into the device menu. Bringing the old window forward instead left the person with the device
# they were trying to change and no menu. Closing that launcher is safe because its engine and
# bridge are in a job object that dies with it — the device is free by the time the mutex is.
if (-not $owned -and $Reconfigure) {
  Write-Host ''
  Write-Host '  ปิดเอนจินที่เปิดอยู่ เพื่อเปลี่ยนอุปกรณ์เสียง...' -ForegroundColor Yellow
  try {
    $running = [int](Get-Content $pidFile -ErrorAction Stop)
    Stop-Process -Id $running -Force -ErrorAction Stop
  } catch { }
  try { $owned = $launcherMutex.WaitOne(10000) } catch [System.Threading.AbandonedMutexException] { $owned = $true }
  # The job closes the engine a moment after its launcher; give the driver that moment.
  Start-Sleep -Milliseconds 800
}

if (-not $owned) {
  Write-Host ''
  Write-Host '  Tone Studio Engine เปิดอยู่แล้ว — สลับไปที่หน้าต่างเดิม' -ForegroundColor Yellow
  try {
    $running = [int](Get-Content $pidFile -ErrorAction Stop)
    [void](New-Object -ComObject WScript.Shell).AppActivate($running)
  } catch { }
  Start-Sleep -Seconds 2
  exit 0
}
New-Item -ItemType Directory -Force -Path (Split-Path $pidFile) | Out-Null
Set-Content -Path $pidFile -Value $PID

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
  # Two sections are read. The first, [Windows Audio], is what the engine opens when
  # --device-type is not given; the other Windows sections list the same hardware under
  # driver-specific names and are skipped. [ASIO] is read as well, because a multi-channel
  # interface (X32, Wing, Dante, Focusrite, RME) shows all of its inputs only there.
  # `(default)` after a name is the device Windows itself is using.
  $lines = & $engine --list-devices 2>$null
  $inputs = @(); $outputs = @(); $asio = @(); $defaultOut = $null; $section = 0; $name = ''
  foreach ($line in $lines) {
    if ($line -match '^\[(.+)\]') { $section += 1; $name = $Matches[1]; continue }
    if ($section -eq 1) {
      if ($line -match '--input\s+"(.+)"') { $inputs += $Matches[1] }
      elseif ($line -match '--output\s+"(.+)"') {
        $outputs += $Matches[1]
        if ($line -match '\(default\)\s*$') { $defaultOut = $outputs[-1] }
      }
    } elseif ($name -eq 'ASIO') {
      # One ASIO driver is one device for both directions, so the two lists are one list.
      if ($line -match '--(input|output)\s+"(.+)"' -and $asio -notcontains $Matches[2]) { $asio += $Matches[2] }
    }
  }
  return @{ Inputs = $inputs; Outputs = $outputs; DefaultOutput = $defaultOut; Asio = $asio }
}

# A name that looks like an audio interface rather than a laptop's own sound chip.
function Test-Interface([string]$name) {
  if ($name -match 'Synaptics|Realtek|Intel|Conexant|High Definition|Default|Primary') { return $false }
  return $name -match 'USB|Focusrite|Scarlett|Behringer|X-USB|XR1|Yamaha|Steinberg|Audient|MOTU|RME|PreSonus|Zoom|Roland|Tascam|Interface'
}

# $preferred, when given and present, is the recommendation; otherwise the first name that looks
# like an interface is.
function Select-Device([string]$title, [string[]]$names, [string]$preferred = '') {
  if ($names.Count -eq 0) { return $null }
  $recommended = 0
  $at = if ($preferred) { [array]::IndexOf($names, $preferred) } else { -1 }
  if ($at -ge 0) { $recommended = $at }
  else { for ($i = 0; $i -lt $names.Count; $i++) { if (Test-Interface $names[$i]) { $recommended = $i; break } } }
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

# `type` is '' for Windows Audio — what a devices.json written before ASIO existed means, since
# it has no type field at all — or 'ASIO'.
function Save-Config($inputName, $outputName, $type) {
  New-Item -ItemType Directory -Force -Path $configDir | Out-Null
  @{ input = $inputName; output = $outputName; type = $type } | ConvertTo-Json | Set-Content -Path $configFile -Encoding UTF8
}

# Which physical device a Windows endpoint belongs to. Windows names both halves of one card
# after the card — "Microphone (USB-Audio)" and "Speakers (USB-Audio)" — so the last bracket is
# the card. A name with no bracket is its own device.
function Get-DeviceKey([string]$name) {
  $found = [regex]::Matches($name, '\(([^()]+)\)')
  if ($found.Count -gt 0) { return $found[$found.Count - 1].Groups[1].Value.Trim().ToLowerInvariant() }
  return $name.Trim().ToLowerInvariant()
}

# The output to recommend for an input: the same card's own output first — the engine needs
# one clock for both directions, and two cards are two clocks — then the speakers Windows is
# already playing through.
#
# Reported from v1.0.6, which recommended Windows' default alone: with the Tank-G as input it
# offered the laptop's headphones, the engine opened the pair at the laptop's 44.1 kHz, and the
# Tank-G's input delivered nothing — every meter at −∞ on a running engine.
function Get-RecommendedOutput([string]$inputName, $devices) {
  $key = Get-DeviceKey $inputName
  foreach ($out in $devices.Outputs) { if ((Get-DeviceKey $out) -eq $key) { return $out } }
  return $devices.DefaultOutput
}

function Test-SameCard([string]$inputName, [string]$outputName) {
  return (Get-DeviceKey $inputName) -eq (Get-DeviceKey $outputName)
}

function Warn-SplitCards([string]$inputName, [string]$outputName) {
  if (Test-SameCard $inputName $outputName) { return }
  Say '  คำเตือน: เสียงเข้าและเสียงออกเป็นคนละการ์ด — นาฬิกาของสองการ์ดไม่ตรงกัน' 'DarkYellow'
  Say '  เสียงอาจไม่เข้าเลย หรือกระตุกเป็นช่วง แนะนำให้ใช้การ์ดเดียวกันทั้งเข้าและออก' 'DarkYellow'
  Say '  (เปลี่ยนได้ที่ Change Audio Device ใน Start Menu)' 'DarkYellow'
  Say ''
}

$AsioLabel = 'ASIO — interface หลายขา (X32, Wing, Dante, Focusrite ...) เห็นทุกขาแยกกัน'
$WasapiLabel = 'Windows Audio — แบบเดิม'

function Choose-Devices {
  $devices = Get-Devices
  Say '  เลือกอุปกรณ์เสียง (ครั้งเดียว — ระบบจะจำไว้)' 'Yellow'
  Say ''

  # The driver question is asked only when an ASIO driver exists. On a machine without one the
  # menu is exactly what it always was.
  $type = ''
  if ($devices.Asio.Count -gt 0) {
    $mode = Select-Device 'ไดรเวอร์เสียง:' @($AsioLabel, $WasapiLabel) $AsioLabel
    Say ''
    if ($mode -eq $AsioLabel) { $type = 'ASIO' }
  }

  if ($type -eq 'ASIO') {
    # One ASIO driver is the input and the output at once, so it is asked for once.
    $in = Select-Device 'ไดรเวอร์ ASIO · ทั้งเสียงเข้าและเสียงออก:' $devices.Asio
    $out = $in
  } else {
    $in = Select-Device 'อินพุต · เสียงเข้าหาเอนจิน:' $devices.Inputs
    Say ''
    $out = Select-Device 'เอาต์พุต · เสียงออกลำโพง:' $devices.Outputs (Get-RecommendedOutput $in $devices)
  }
  Say ''
  if ($in -and $out -and $type -ne 'ASIO') { Warn-SplitCards $in $out }
  if (-not $in -or -not $out) {
    Say '  ไม่พบอุปกรณ์เสียงในเครื่อง — เสียบ interface แล้วเปิดใหม่' 'Red'
    Read-Host '  กด Enter เพื่อปิด'
    exit 1
  }
  Save-Config $in $out $type
  Say '  บันทึกแล้ว — ครั้งต่อไปเปิดแล้วทำงานทันที' 'Green'
  Say ''
  return @{ input = $in; output = $out; type = $type }
}

$config = Read-Config
if (-not $config) { $config = Choose-Devices }

# Wait for the saved devices rather than failing: an icon clicked before the USB cable is in
# should start the moment it is, not print an error.
function Wait-ForDevices($config) {
  $warned = $false
  while ($true) {
    $devices = Get-Devices
    if ($config.type -eq 'ASIO') {
      $hasIn = $devices.Asio -contains $config.input
      $hasOut = $hasIn
    } else {
      $hasIn = $devices.Inputs -contains $config.input
      $hasOut = $devices.Outputs -contains $config.output
    }
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
#
# Reported from the first real install: closing the window left the engine running with no
# window, and the launcher kept restarting it — two sets, both sending sound to the Tank-G, a
# howl that "closing the engine" could not stop. Three rules now:
#
#   - The engine and the bridge are in a Windows job object that dies with this window, so
#     closing it — X, Ctrl+C, logging off — stops them. Nothing outlives the window.
#   - The engine is restarted only when it crashes, and not in a tight loop: three failures
#     inside ten seconds each is a device or a name problem, and it stops and says so.
#   - One launcher at a time (the mutex at the top of this file).
# ---------------------------------------------------------------------------------------------
Add-Type -TypeDefinition @"
using System;
using System.Runtime.InteropServices;
public static class ToneStudioJob {
  [DllImport("kernel32.dll", CharSet = CharSet.Unicode)] static extern IntPtr CreateJobObject(IntPtr attributes, string name);
  [DllImport("kernel32.dll")] static extern bool SetInformationJobObject(IntPtr job, int infoClass, IntPtr info, uint length);
  [DllImport("kernel32.dll")] static extern bool AssignProcessToJobObject(IntPtr job, IntPtr process);
  [StructLayout(LayoutKind.Sequential)] struct Basic { public long a; public long b; public uint LimitFlags; public UIntPtr c; public UIntPtr d; public uint e; public UIntPtr f; public uint g; public uint h; }
  [StructLayout(LayoutKind.Sequential)] struct Io { public ulong a, b, c, d, e, f; }
  [StructLayout(LayoutKind.Sequential)] struct Extended { public Basic BasicInfo; public Io IoInfo; public UIntPtr p1, p2, p3, p4; }
  static IntPtr job = IntPtr.Zero;
  public static void Init() {
    job = CreateJobObject(IntPtr.Zero, null);
    var info = new Extended();
    info.BasicInfo.LimitFlags = 0x2000; // JOB_OBJECT_LIMIT_KILL_ON_JOB_CLOSE
    int length = Marshal.SizeOf(typeof(Extended));
    IntPtr ptr = Marshal.AllocHGlobal(length);
    Marshal.StructureToPtr(info, ptr, false);
    SetInformationJobObject(job, 9, ptr, (uint)length); // JobObjectExtendedLimitInformation
    Marshal.FreeHGlobal(ptr);
  }
  public static bool Add(IntPtr process) { return AssignProcessToJobObject(job, process); }
}
"@
[ToneStudioJob]::Init()

# Leftovers from a launcher that did not have the job (v1.0.0): its orphaned engine still holds
# the audio device and port 8080, and its own launcher would restart it behind our back.
$oldLaunchers = Get-CimInstance Win32_Process -Filter "Name='powershell.exe'" -ErrorAction SilentlyContinue |
  Where-Object { $_.ProcessId -ne $PID -and $_.CommandLine -match 'start-engine\.ps1' }
$stale = Get-Process -Name 'tone-studio-app', 'tone-studio-bridge' -ErrorAction SilentlyContinue
if ($oldLaunchers -or $stale) {
  Say '  ปิดเอนจิน/บริดจ์ตัวเก่าที่ยังค้างอยู่...' 'DarkGray'
  $oldLaunchers | ForEach-Object { Stop-Process -Id $_.ProcessId -Force -ErrorAction SilentlyContinue }
  Start-Sleep -Milliseconds 300
  Get-Process -Name 'tone-studio-app', 'tone-studio-bridge' -ErrorAction SilentlyContinue | Stop-Process -Force -ErrorAction SilentlyContinue
  Start-Sleep -Milliseconds 500
}

$bridgeArgs = @{ FilePath = $bridge.File; WindowStyle = 'Minimized'; PassThru = $true }
if ($bridge.Args.Count -gt 0) { $bridgeArgs.ArgumentList = $bridge.Args }
$bridgeProcess = Start-Process @bridgeArgs
[void][ToneStudioJob]::Add($bridgeProcess.Handle)
Say '  บริดจ์ทำงานแล้ว · ws://127.0.0.1:8080' 'Green'

$quickFailures = 0
while ($true) {
  $config = Wait-ForDevices $config
  Say ''
  Say "  อินพุต   : $($config.input)" 'White'
  Say "  เอาต์พุต : $($config.output)" 'White'
  if ($config.type -eq 'ASIO') { Say '  ไดรเวอร์ : ASIO' 'White' }
  else { Say ''; Warn-SplitCards $config.input $config.output }
  Say '  เอนจินกำลังทำงาน — ย่อหน้าต่างนี้ได้ · ปิดหน้าต่างนี้ = หยุดเอนจินและบริดจ์ทันที' 'Green'
  Say ''

  $engineArgs = @('--input', ('"' + $config.input + '"'), '--output', ('"' + $config.output + '"'))
  if ($config.type -eq 'ASIO') { $engineArgs += @('--device-type', 'ASIO') }

  $started = Get-Date
  $engineProcess = Start-Process -FilePath $engine -NoNewWindow -PassThru -ArgumentList $engineArgs
  [void][ToneStudioJob]::Add($engineProcess.Handle) # also caches the handle, so ExitCode is readable
  $engineProcess.WaitForExit()
  $code = $engineProcess.ExitCode
  $ranFor = ((Get-Date) - $started).TotalSeconds

  if ($code -eq 0) {
    Say ''
    Say '  เอนจินปิดแล้ว' 'DarkYellow'
    break
  }
  if ($ranFor -lt 10) { $quickFailures += 1 } else { $quickFailures = 0 }
  if ($quickFailures -ge 3) {
    Say ''
    Say "  เอนจินเปิดไม่ขึ้นซ้ำๆ (code $code) — เช็กว่าเสียบอุปกรณ์แล้ว หรือกด Change Audio Device ใน Start Menu" 'Red'
    Read-Host '  กด Enter เพื่อปิด'
    break
  }
  Say ''
  Say "  เอนจินหยุดเอง (code $code) — เปิดใหม่ใน 3 วินาที · ปิดหน้าต่างนี้เพื่อหยุด" 'DarkYellow'
  Start-Sleep -Seconds 3
}

if (-not $bridgeProcess.HasExited) { Stop-Process -Id $bridgeProcess.Id -Force -ErrorAction SilentlyContinue }
