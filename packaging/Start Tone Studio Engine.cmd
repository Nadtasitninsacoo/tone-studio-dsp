@echo off
rem The Desktop icon points here. PowerShell is started with the execution policy bypassed for
rem this one script only, because Windows' default policy refuses unsigned .ps1 files and the
rem customer should never have to change a system setting to press an icon.
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0start-engine.ps1" %*
