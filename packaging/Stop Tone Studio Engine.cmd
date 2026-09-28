@echo off
rem Stop the engine, the bridge and every launcher, in one click.
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0stop-engine.ps1"
