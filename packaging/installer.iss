; Tone Studio Engine — Windows installer.
;
; Built by packaging\build-installer.ps1, which stages every file into dist\stage first. The
; customer gets one Setup.exe: no Git, no CMake, no Visual Studio, no Node.js.
;
;  - Per-user install (PrivilegesRequired=lowest): no UAC prompt, and the launcher's saved
;    device choice lives in %APPDATA% beside it. A machine-wide install is offered by the dialog.
;  - The Desktop icon runs the launcher, which picks devices once and remembers them.
;  - "Start with Windows" is off by default: an engine that opens the audio interface on every
;    boot is a decision about somebody's machine, and it is one tick away.

#ifndef AppVersion
  #define AppVersion "1.0.0"
#endif

[Setup]
AppId={{6C2E7B1D-9E4A-4F7B-B7C1-3A5D0E2F9A10}
AppName=Tone Studio Engine
AppVersion={#AppVersion}
AppPublisher=Nadtasit Keng
AppPublisherURL=https://github.com/Nadtasitninsacoo/tone-studio-dsp
AppSupportURL=https://github.com/Nadtasitninsacoo/tone-studio-dsp
DefaultDirName={autopf}\Tone Studio Engine
DefaultGroupName=Tone Studio Engine
DisableProgramGroupPage=yes
PrivilegesRequired=lowest
PrivilegesRequiredOverridesAllowed=dialog
OutputDir=..\dist
OutputBaseFilename=ToneStudioEngine-Setup-{#AppVersion}
SetupIconFile=icon.ico
UninstallDisplayIcon={app}\icon.ico
LicenseFile=..\LICENSE
Compression=lzma2/max
SolidCompression=yes
WizardStyle=modern
ArchitecturesAllowed=x64compatible
ArchitecturesInstallIn64BitMode=x64compatible
CloseApplications=yes

[Tasks]
Name: "desktopicon"; Description: "Create a Desktop icon"; GroupDescription: "Shortcuts:"
Name: "startup"; Description: "Start the engine automatically when Windows starts"; GroupDescription: "Startup:"; Flags: unchecked

[Files]
Source: "..\dist\stage\*"; DestDir: "{app}"; Flags: ignoreversion recursesubdirs

[Icons]
Name: "{autodesktop}\Tone Studio Engine"; Filename: "{app}\Start Tone Studio Engine.cmd"; WorkingDir: "{app}"; IconFilename: "{app}\icon.ico"; Tasks: desktopicon
Name: "{group}\Tone Studio Engine"; Filename: "{app}\Start Tone Studio Engine.cmd"; WorkingDir: "{app}"; IconFilename: "{app}\icon.ico"
Name: "{group}\Change Audio Device"; Filename: "{app}\Change Audio Device.cmd"; WorkingDir: "{app}"; IconFilename: "{app}\icon.ico"
Name: "{group}\Stop Tone Studio Engine"; Filename: "{app}\Stop Tone Studio Engine.cmd"; WorkingDir: "{app}"; IconFilename: "{app}\icon.ico"
Name: "{group}\Uninstall Tone Studio Engine"; Filename: "{uninstallexe}"
Name: "{userstartup}\Tone Studio Engine"; Filename: "{app}\Start Tone Studio Engine.cmd"; WorkingDir: "{app}"; IconFilename: "{app}\icon.ico"; Tasks: startup

[Run]
Filename: "{app}\Start Tone Studio Engine.cmd"; Description: "Start Tone Studio Engine now"; Flags: postinstall nowait skipifsilent shellexec

[UninstallRun]
; Stop a running engine and bridge so their files can be removed.
Filename: "powershell.exe"; Parameters: "-NoProfile -ExecutionPolicy Bypass -File ""{app}\stop-engine.ps1"""; Flags: runhidden; RunOnceId: "StopEngine"
