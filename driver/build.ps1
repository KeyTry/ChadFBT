$ErrorActionPreference = 'Stop'
$vswhere = Join-Path ${env:ProgramFiles(x86)} 'Microsoft Visual Studio\Installer\vswhere.exe'
$vs = & $vswhere -latest -products '*' -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath
if (-not $vs) { throw 'Visual Studio C++ tools are required.' }
$developer = Join-Path $vs 'VC\Auxiliary\Build\vcvars64.bat'
$root = $PSScriptRoot
New-Item -ItemType Directory -Force (Join-Path $root 'build'), (Join-Path $root 'chadfbt\bin\win64') | Out-Null
Push-Location $root
try {
 # cl defaults to the static CRT (/MT); CMakeLists.txt selects the same runtime.
 $cl = 'cl /nologo /std:c++17 /EHsc /W4 /O2'
 $commands = 'call "' + $developer + '" >nul && ' + $cl + ' /LD /I third_party\openvr /I src src\driver.cpp /Fo:build\driver.obj /link /OUT:chadfbt\bin\win64\driver_chadfbt.dll /IMPLIB:build\driver.lib ws2_32.lib && ' + $cl + ' /I third_party\openvr /I src src\anchors.cpp /Fo:build\anchors.obj /link /OUT:build\chadfbt-anchors.exe ws2_32.lib && ' + $cl + ' /I src tests\wire_tests.cpp /Fo:build\wire_tests.obj /link /OUT:build\wire-tests.exe && build\wire-tests.exe tests\fixtures\pose-validity.txt tests\fixtures\anchor-v2.hex && ' + $cl + ' /I src /I third_party\openvr tests\receiver_tests.cpp /Fo:build\receiver_tests.obj /link /OUT:build\receiver-tests.exe && build\receiver-tests.exe tests\fixtures\target-v1.hex tests\fixtures\target-v2.hex'
 $commands += ' && ' + $cl + ' /I src /I third_party\openvr tests\tracker_role_tests.cpp /Fo:build\tracker_role_tests.obj /link /OUT:build\tracker-role-tests.exe && build\tracker-role-tests.exe'
 & cmd.exe /d /c $commands
 if ($LASTEXITCODE -ne 0) { throw "Native build failed with exit code $LASTEXITCODE. Install the Windows SDK and MSVC x64 tools." }
 Copy-Item -LiteralPath 'third_party\openvr\openvr_api.dll' -Destination 'build\openvr_api.dll' -Force
} finally { Pop-Location }
