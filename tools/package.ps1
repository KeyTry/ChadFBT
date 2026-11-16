$ErrorActionPreference = 'Stop'
$root = Split-Path $PSScriptRoot -Parent
$bundle = Join-Path $root 'dist\ChadFBT'
$required = @('target\x86_64-pc-windows-msvc\release\chadfbt.exe', 'driver\build\chadfbt-anchors.exe', 'driver\build\openvr_api.dll', 'driver\chadfbt\bin\win64\driver_chadfbt.dll', 'driver\PROTOCOL.md', 'driver\FIRMWARE-TIMING.md')
foreach ($file in $required) { if (-not (Test-Path -LiteralPath (Join-Path $root $file))) { throw "Missing $file. Run tools\build.ps1 first." } }
New-Item -ItemType Directory -Force $bundle | Out-Null
Copy-Item -LiteralPath (Join-Path $root 'target\x86_64-pc-windows-msvc\release\chadfbt.exe') -Destination $bundle -Force
Copy-Item -LiteralPath (Join-Path $root 'driver\build\chadfbt-anchors.exe') -Destination $bundle -Force
Copy-Item -LiteralPath (Join-Path $root 'driver\build\openvr_api.dll') -Destination $bundle -Force
New-Item -ItemType Directory -Force (Join-Path $bundle 'driver') | Out-Null
Copy-Item -LiteralPath (Join-Path $root 'driver\chadfbt') -Destination (Join-Path $bundle 'driver') -Recurse -Force
Copy-Item -LiteralPath (Join-Path $root 'driver\registration.ps1') -Destination (Join-Path $bundle 'driver') -Force
Copy-Item -LiteralPath (Join-Path $root 'driver\PROTOCOL.md') -Destination (Join-Path $bundle 'driver') -Force
Copy-Item -LiteralPath (Join-Path $root 'driver\FIRMWARE-TIMING.md') -Destination (Join-Path $bundle 'driver') -Force
Copy-Item -LiteralPath (Join-Path $root 'README.md') -Destination $bundle -Force
Copy-Item -LiteralPath (Join-Path $root 'docs') -Destination $bundle -Recurse -Force
New-Item -ItemType Directory -Force (Join-Path $bundle 'crates\tracking-core') | Out-Null
Copy-Item -LiteralPath (Join-Path $root 'crates\tracking-core\README.md') -Destination (Join-Path $bundle 'crates\tracking-core') -Force
Copy-Item -LiteralPath (Join-Path $root 'driver\third_party\openvr\LICENSE') -Destination (Join-Path $bundle 'OPENVR-LICENSE.txt') -Force
foreach ($name in 'LICENSE-MIT', 'LICENSE-APACHE') {
    $license = Join-Path $root $name
    if (Test-Path -LiteralPath $license) { Copy-Item -LiteralPath $license -Destination $bundle -Force }
}
$cargo = Join-Path $env:USERPROFILE '.cargo\bin\cargo.exe'
if (-not (Test-Path -LiteralPath $cargo)) { $cargo = (Get-Command cargo -ErrorAction Stop).Source }
$metadataPath = Join-Path $root 'local-data\cargo-metadata.json'
New-Item -ItemType Directory -Force (Split-Path $metadataPath -Parent) | Out-Null
& $cargo metadata --manifest-path (Join-Path $root 'Cargo.toml') --format-version 1 --filter-platform x86_64-pc-windows-msvc --offline | Set-Content -LiteralPath $metadataPath -Encoding utf8
if ($LASTEXITCODE -ne 0) { throw 'Dependency metadata could not be generated.' }
python (Join-Path $PSScriptRoot 'collect-licenses.py') $metadataPath (Join-Path $bundle 'third-party-licenses')
if ($LASTEXITCODE -ne 0) { throw 'Dependency notice collection failed.' }
Get-ChildItem -LiteralPath $bundle -File -Recurse | Where-Object { $_.Name -ne 'checksums.json' } | ForEach-Object {
    [pscustomobject]@{ Path = $_.FullName.Substring($bundle.Length + 1); SHA256 = (Get-FileHash -LiteralPath $_.FullName -Algorithm SHA256).Hash }
} | ConvertTo-Json | Set-Content -LiteralPath (Join-Path $bundle 'checksums.json')
Write-Output "Portable bundle: $bundle"
