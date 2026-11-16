param([ValidateSet("install", "uninstall", "status")][string]$Action = "status", [string]$VrPathReg = "")
$ErrorActionPreference = 'Stop'
if (-not $VrPathReg) { $VrPathReg = Join-Path ${env:ProgramFiles(x86)} 'Steam\steamapps\common\SteamVR\bin\win64\vrpathreg.exe' }
if (-not (Test-Path -LiteralPath $VrPathReg -PathType Leaf)) { throw "vrpathreg.exe not found; pass -VrPathReg with its full path." }
$driver = (Resolve-Path -LiteralPath (Join-Path $PSScriptRoot 'chadfbt')).Path
if ($Action -eq 'install') { if (-not (Test-Path -LiteralPath (Join-Path $driver 'bin\win64\driver_chadfbt.dll'))) { throw 'Build the driver first.' }; & $VrPathReg adddriver $driver }
elseif ($Action -eq 'uninstall') { & $VrPathReg removedriver $driver }
else { & $VrPathReg finddriver chadfbt }
if ($LASTEXITCODE -ne 0) { throw "vrpathreg failed with exit code $LASTEXITCODE" }
