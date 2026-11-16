# Captures the running ChadFBT window to a PNG without needing it in the foreground.
# Development aid for visual UI checks; it does not interact with tracking.
#
#   .\tools\capture-window.ps1 -Out local-data\ui.png
#   .\tools\capture-window.ps1 -Out local-data\ui.png -Click 289,221      # click first (window coordinates)
#   .\tools\capture-window.ps1 -Out local-data\ui.png -Scroll 250,600,-10 # x, y, wheel notches (negative = down)
#   .\tools\capture-window.ps1 -Out local-data\crop.png -Crop 30,180,460,830
param(
    [Parameter(Mandatory = $true)][string]$Out,
    [int[]]$Click,
    [int[]]$Scroll,
    [int[]]$Crop,
    [string]$ProcessName = 'chadfbt',
    [int]$SettleMs = 700
)
$ErrorActionPreference = 'Stop'
Add-Type -AssemblyName System.Drawing
Add-Type @'
using System;
using System.Runtime.InteropServices;
public class ChadFbtWindow {
    [DllImport("user32.dll")] public static extern bool GetWindowRect(IntPtr h, out RECT r);
    [DllImport("user32.dll")] public static extern bool SetForegroundWindow(IntPtr h);
    [DllImport("user32.dll")] public static extern bool SetProcessDPIAware();
    [DllImport("user32.dll")] public static extern bool SetCursorPos(int x, int y);
    [DllImport("user32.dll")] public static extern void mouse_event(uint f, uint dx, uint dy, int data, UIntPtr e);
    [DllImport("user32.dll")] public static extern bool PrintWindow(IntPtr h, IntPtr dc, uint f);
    public struct RECT { public int L, T, R, B; }
}
'@
[ChadFbtWindow]::SetProcessDPIAware() | Out-Null
$process = Get-Process $ProcessName -ErrorAction SilentlyContinue | Where-Object { $_.MainWindowHandle -ne 0 } | Select-Object -First 1
if (-not $process) { throw "No visible '$ProcessName' window found." }
$handle = $process.MainWindowHandle
$rect = New-Object ChadFbtWindow+RECT
[ChadFbtWindow]::GetWindowRect($handle, [ref]$rect) | Out-Null

if ($Click -or $Scroll) {
    [ChadFbtWindow]::SetForegroundWindow($handle) | Out-Null
    Start-Sleep -Milliseconds 300
}
if ($Scroll) {
    if ($Scroll.Count -ne 3) { throw '-Scroll needs x,y,notches' }
    [ChadFbtWindow]::SetCursorPos($rect.L + $Scroll[0], $rect.T + $Scroll[1]) | Out-Null
    Start-Sleep -Milliseconds 150
    $direction = if ($Scroll[2] -lt 0) { -120 } else { 120 }
    1..[Math]::Abs($Scroll[2]) | ForEach-Object {
        [ChadFbtWindow]::mouse_event(0x0800, 0, 0, $direction, [UIntPtr]::Zero)
        Start-Sleep -Milliseconds 40
    }
}
if ($Click) {
    if ($Click.Count -ne 2) { throw '-Click needs x,y' }
    [ChadFbtWindow]::SetCursorPos($rect.L + $Click[0], $rect.T + $Click[1]) | Out-Null
    Start-Sleep -Milliseconds 200
    [ChadFbtWindow]::mouse_event(0x0002, 0, 0, 0, [UIntPtr]::Zero)
    [ChadFbtWindow]::mouse_event(0x0004, 0, 0, 0, [UIntPtr]::Zero)
}
if ($Click -or $Scroll) { Start-Sleep -Milliseconds $SettleMs }

$width = $rect.R - $rect.L
$height = $rect.B - $rect.T
$bitmap = New-Object System.Drawing.Bitmap $width, $height
$graphics = [System.Drawing.Graphics]::FromImage($bitmap)
$dc = $graphics.GetHdc()
[ChadFbtWindow]::PrintWindow($handle, $dc, 2) | Out-Null
$graphics.ReleaseHdc($dc)
$graphics.Dispose()

$target = $bitmap
if ($Crop) {
    if ($Crop.Count -ne 4) { throw '-Crop needs x,y,width,height' }
    $area = New-Object System.Drawing.Rectangle -ArgumentList $Crop[0], $Crop[1], $Crop[2], $Crop[3]
    $target = $bitmap.Clone($area, $bitmap.PixelFormat)
}
$outPath = if ([IO.Path]::IsPathRooted($Out)) { $Out } else { Join-Path (Get-Location) $Out }
New-Item -ItemType Directory -Force (Split-Path $outPath -Parent) | Out-Null
$target.Save($outPath, [System.Drawing.Imaging.ImageFormat]::Png)
if ($Crop) { $target.Dispose() }
$bitmap.Dispose()
Write-Output "Saved $outPath (${width}x${height} window)"
