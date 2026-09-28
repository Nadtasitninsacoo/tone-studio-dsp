<#
  Build Setup.exe for Tone Studio Engine.

    powershell -ExecutionPolicy Bypass -File packaging\build-installer.ps1

  Steps: build the engine (Release, static runtime), turn bridge.js into a standalone
  tone-studio-bridge.exe with Node's single-executable support, stage everything in
  dist\stage, and compile packaging\installer.iss with Inno Setup. The result is
  dist\ToneStudioEngine-Setup-<version>.exe — upload that to a GitHub Release.

  Needs on the BUILD machine only: CMake, VS 2022 Build Tools, Node.js 20+, Inno Setup 6.
  The customer needs none of them.

  -SkipBuild  reuse the engine already in build\ (faster when only packaging changed).
#>
param([switch]$SkipBuild)

$ErrorActionPreference = 'Stop'
$root = Split-Path -Parent (Split-Path -Parent $MyInvocation.MyCommand.Path)
$pack = Join-Path $root 'packaging'
$dist = Join-Path $root 'dist'
$stage = Join-Path $dist 'stage'
Set-Location $root

function Step([string]$text) { Write-Host "==> $text" -ForegroundColor Cyan }

# The version is the one CMake builds with, so the installer and the binary cannot disagree.
$cmakeText = Get-Content (Join-Path $root 'CMakeLists.txt') -Raw
if ($cmakeText -notmatch 'project\(tone-studio-dsp VERSION ([0-9.]+)') { throw 'No version in CMakeLists.txt' }
$version = $Matches[1]
Step "Tone Studio Engine $version"

if (-not $SkipBuild) {
  Step 'Building the engine (Release)'
  cmake -B build | Out-Host
  if ($LASTEXITCODE -ne 0) { throw 'cmake configure failed' }
  cmake --build build --config Release -- -m | Out-Host
  if ($LASTEXITCODE -ne 0) { throw 'cmake build failed' }
}
$engine = Join-Path $root 'build\app\tone-studio-app_artefacts\Release\tone-studio-app.exe'
if (-not (Test-Path $engine)) { throw "Engine not found at $engine" }

Step 'Staging files'
if (Test-Path $stage) { Remove-Item $stage -Recurse -Force }
New-Item -ItemType Directory -Force -Path $stage | Out-Null
Copy-Item $engine $stage
Copy-Item (Join-Path $pack 'start-engine.ps1') $stage
Copy-Item (Join-Path $pack 'Start Tone Studio Engine.cmd') $stage
Copy-Item (Join-Path $pack 'Change Audio Device.cmd') $stage
Copy-Item (Join-Path $pack 'icon.ico') $stage
Copy-Item (Join-Path $root 'LICENSE') (Join-Path $stage 'LICENSE.txt')
Copy-Item (Join-Path $root 'README.md') $stage

Step 'Bundling the bridge into tone-studio-bridge.exe (Node single executable)'
$node = (Get-Command node).Source
$blob = Join-Path $dist 'sea-prep.blob'
$seaConfig = Join-Path $dist 'sea-config.json'
@{ main = (Join-Path $root 'bridge.js'); output = $blob; disableExperimentalSEAWarning = $true } |
  ConvertTo-Json | Set-Content -Path $seaConfig -Encoding ASCII
& $node --experimental-sea-config $seaConfig | Out-Host
if ($LASTEXITCODE -ne 0) { throw 'SEA blob failed' }
$bridgeExe = Join-Path $stage 'tone-studio-bridge.exe'
Copy-Item $node $bridgeExe
& npx --yes postject $bridgeExe NODE_SEA_BLOB $blob --sentinel-fuse NODE_SEA_FUSE_fce680ab2cc467b6e072b8b5df1996b2 | Out-Host
if ($LASTEXITCODE -ne 0) { throw 'postject failed' }

Step 'Checking the staged binaries start'
$list = & $engine --list-devices 2>&1 | Out-String
if ($list -notmatch 'Audio devices on this machine') { throw "Engine did not list devices:`n$list" }

Step 'Compiling the installer'
$iscc = @(
  (Join-Path $env:LOCALAPPDATA 'Programs\Inno Setup 6\ISCC.exe'),
  'C:\Program Files (x86)\Inno Setup 6\ISCC.exe',
  'C:\Program Files\Inno Setup 6\ISCC.exe'
) | Where-Object { Test-Path $_ } | Select-Object -First 1
if (-not $iscc) { throw 'Inno Setup 6 not found — winget install JRSoftware.InnoSetup' }
& $iscc "/DAppVersion=$version" (Join-Path $pack 'installer.iss') | Out-Host
if ($LASTEXITCODE -ne 0) { throw 'Inno Setup failed' }

$setup = Join-Path $dist "ToneStudioEngine-Setup-$version.exe"
Step "Done: $setup"
