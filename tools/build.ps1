param([switch]$DebugBuild)
$ErrorActionPreference = 'Stop'
$root = Split-Path $PSScriptRoot -Parent
$cargo = Join-Path $env:USERPROFILE '.cargo\bin\cargo.exe'
if (-not (Test-Path -LiteralPath $cargo)) { $cargo = (Get-Command cargo -ErrorAction Stop).Source }
Push-Location $root
$oldFlags = $env:RUSTFLAGS
$oldEncodedFlags = $env:CARGO_ENCODED_RUSTFLAGS
try {
    if (-not $DebugBuild) {
        # Rust embeds the absolute path of every source file it compiled (panic
        # locations), which would put the builder's user name and directory layout
        # into a released binary. Remap them to neutral names. The flags are passed
        # through CARGO_ENCODED_RUSTFLAGS so paths containing spaces survive.
        $cargoHome = if ($env:CARGO_HOME) { $env:CARGO_HOME } else { Join-Path $env:USERPROFILE '.cargo' }
        $rustupHome = if ($env:RUSTUP_HOME) { $env:RUSTUP_HOME } else { Join-Path $env:USERPROFILE '.rustup' }
        $flags = @(
            '-C', 'target-feature=+crt-static',
            "--remap-path-prefix=$root=chadfbt",
            "--remap-path-prefix=$cargoHome=cargo",
            "--remap-path-prefix=$rustupHome=rustup",
            "--remap-path-prefix=$env:USERPROFILE=home"
        )
        $env:RUSTFLAGS = $null
        $env:CARGO_ENCODED_RUSTFLAGS = $flags -join [char]0x1f
    }
    & (Join-Path $root 'driver\build.ps1')
    if ($LASTEXITCODE -ne 0) { throw 'Driver build failed' }
    if ($DebugBuild) { & $cargo build -p chadfbt } else { & $cargo build --release --target x86_64-pc-windows-msvc -p chadfbt }
    if ($LASTEXITCODE -ne 0) { throw 'Rust application build failed' }
} finally {
    $env:RUSTFLAGS = $oldFlags
    $env:CARGO_ENCODED_RUSTFLAGS = $oldEncodedFlags
    Pop-Location
}
