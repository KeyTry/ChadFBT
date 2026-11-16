param([switch]$DebugBuild)
$ErrorActionPreference = 'Stop'
$root = Split-Path $PSScriptRoot -Parent
$cargo = Join-Path $env:USERPROFILE '.cargo\bin\cargo.exe'
if (-not (Test-Path -LiteralPath $cargo)) { $cargo = (Get-Command cargo -ErrorAction Stop).Source }
Push-Location $root
$oldFlags = $env:RUSTFLAGS
try {
    if (-not $DebugBuild) { $env:RUSTFLAGS = '-C target-feature=+crt-static' }
    & (Join-Path $root 'driver\build.ps1')
    if ($LASTEXITCODE -ne 0) { throw 'Driver build failed' }
    if ($DebugBuild) { & $cargo build -p chadfbt } else { & $cargo build --release --target x86_64-pc-windows-msvc -p chadfbt }
    if ($LASTEXITCODE -ne 0) { throw 'Rust application build failed' }
} finally {
    $env:RUSTFLAGS = $oldFlags
    Pop-Location
}
