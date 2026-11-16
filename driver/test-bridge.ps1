$ErrorActionPreference = 'Stop'
$vswhere = Join-Path ${env:ProgramFiles(x86)} 'Microsoft Visual Studio\Installer\vswhere.exe'
$vs = & $vswhere -latest -products '*' -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath
if (-not $vs) { throw 'Visual Studio C++ tools and a Windows SDK are required.' }
$developer = Join-Path $vs 'VC\Auxiliary\Build\vcvars64.bat'
$cargo = Get-Command cargo.exe -ErrorAction SilentlyContinue | Select-Object -First 1 -ExpandProperty Source
if (-not $cargo) { $cargo = Join-Path $env:USERPROFILE '.cargo\bin\cargo.exe' }
if (-not (Test-Path -LiteralPath $cargo -PathType Leaf)) { throw 'Rust cargo.exe was not found.' }
New-Item -ItemType Directory -Force (Join-Path $PSScriptRoot 'build') | Out-Null
Push-Location $PSScriptRoot
try {
    # This compiles only the offline fixture harness; it does not overwrite the registered DLL.
    $commands = 'call "' + $developer + '" >nul && cl /nologo /std:c++17 /EHsc /W4 /O2 /I src tests\wire_tests.cpp /Fo:build\wire_tests.obj /link /OUT:build\wire-tests.exe && build\wire-tests.exe tests\fixtures\pose-validity.txt tests\fixtures\anchor-v2.hex && cl /nologo /std:c++17 /EHsc /W4 /O2 /I src /I third_party\openvr tests\receiver_tests.cpp /Fo:build\receiver_tests.obj /link /OUT:build\receiver-tests.exe && build\receiver-tests.exe tests\fixtures\target-v1.hex tests\fixtures\target-v2.hex'
    & cmd.exe /d /c $commands
    if ($LASTEXITCODE -ne 0) { throw "Native receiver test failed with exit code $LASTEXITCODE." }
} finally { Pop-Location }
Push-Location (Split-Path -Parent $PSScriptRoot)
try {
    $commands = 'call "' + $developer + '" >nul && "' + $cargo + '" test -p steamvr-bridge && "' + $cargo + '" test -p chadfbt runtime::tracking_space_tests'
    & cmd.exe /d /c $commands
    if ($LASTEXITCODE -ne 0) { throw "Rust bridge or app fixture tests failed with exit code $LASTEXITCODE." }
} finally { Pop-Location }
Write-Host 'Offline bridge checks passed. This does not test a live SteamVR tracking universe.'
