@echo off
rem "Change audio device": forget the saved input/output and ask again.
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0start-engine.ps1" -Reconfigure
