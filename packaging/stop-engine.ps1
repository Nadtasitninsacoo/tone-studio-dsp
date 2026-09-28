<#
  Stop Tone Studio Engine — the Start Menu's one-click off switch.

  Stops every launcher first (so nothing restarts the engine), then any engine or bridge left
  over. With the job object a launcher's children die with it; the second sweep is for an
  engine started by something else, or by v1.0.0, which had no job.
#>
try { [Console]::OutputEncoding = [Text.Encoding]::UTF8 } catch { }
$launchers = Get-CimInstance Win32_Process -Filter "Name='powershell.exe'" -ErrorAction SilentlyContinue |
  Where-Object { $_.ProcessId -ne $PID -and $_.CommandLine -match 'start-engine\.ps1' }
$launchers | ForEach-Object { Stop-Process -Id $_.ProcessId -Force -ErrorAction SilentlyContinue }
Start-Sleep -Milliseconds 400
Get-Process -Name 'tone-studio-app', 'tone-studio-bridge' -ErrorAction SilentlyContinue | Stop-Process -Force -ErrorAction SilentlyContinue
Start-Sleep -Milliseconds 400
$left = Get-Process -Name 'tone-studio-app', 'tone-studio-bridge' -ErrorAction SilentlyContinue
if ($left) {
  Write-Host '  ยังปิดไม่หมด — ลองอีกครั้ง หรือรีสตาร์ทเครื่อง' -ForegroundColor Red
} else {
  Write-Host '  หยุด Tone Studio Engine แล้ว — ไม่มีเสียงออกจากเอนจิน' -ForegroundColor Green
}
Start-Sleep -Seconds 2
