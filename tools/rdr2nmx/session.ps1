# Logging session for rdr2nmx: `session.ps1 start -game <RDR2 folder>` copies the logger into RDR2 and moves the listed
# ragdoll / Euphoria mods aside (they change what you'd record); `session.ps1 end -game <RDR2 folder>` takes the logger
# out and puts the mods back. Nothing is deleted. Close RDR2 first.
param([ValidateSet("start", "end", "status")] [string]$what = "status",
      [Parameter(Mandatory = $true)] [string]$game,
      # Mods that change ragdolls, moved aside while logging. Add your own.
      [string[]]$mods = @("PedDamageOverhaul.asi", "PedDamageOverhaul.ini", "Simple Ragdoll.asi", "EnhancedBrawling.asi",
                          "EnhancedBrawling.ini", "Dynamic Shove & Throw.asi", "Dynamic Shove & Throw.ini", "Drag Bodies.asi",
                          "Drag Bodies.ini"))
$aside = Join-Path $game "rdr2nmx_mods_aside"
$asi = Join-Path $PSScriptRoot "out\rdr2nmx.asi"
if ($what -eq "start") {
    if (Get-Process RDR2 -ErrorAction SilentlyContinue) { throw "RDR2 is running: close it first" }
    if (-not (Test-Path -LiteralPath $asi)) { throw "out\rdr2nmx.asi not found: build it or download it first" }
    New-Item -ItemType Directory -Force $aside | Out-Null
    foreach ($m in $mods) { $p = Join-Path $game $m; if (Test-Path -LiteralPath $p) { Move-Item -LiteralPath $p -Destination $aside -Force; "moved aside: $m" } }
    Copy-Item -LiteralPath $asi -Destination (Join-Path $game "rdr2nmx.asi") -Force
    "logger in: $game\rdr2nmx.asi"
} elseif ($what -eq "end") {
    if (Get-Process RDR2 -ErrorAction SilentlyContinue) { throw "RDR2 is running: close it first" }
    $x = Join-Path $game "rdr2nmx.asi"; if (Test-Path -LiteralPath $x) { Remove-Item -LiteralPath $x; "logger out" }
    if (Test-Path -LiteralPath $aside) {
        Get-ChildItem -LiteralPath $aside | ForEach-Object { Move-Item -LiteralPath $_.FullName -Destination $game -Force; "restored: $($_.Name)" }
        Remove-Item -LiteralPath $aside
    }
} else {
    "logger installed: " + (Test-Path -LiteralPath (Join-Path $game "rdr2nmx.asi"))
    "mods aside: " + ((Get-ChildItem -LiteralPath $aside -ErrorAction SilentlyContinue | ForEach-Object Name) -join ", ")
    "recordings: " + ((Get-ChildItem -LiteralPath (Join-Path $game "rdr2nmx") -Filter *.nmx -ErrorAction SilentlyContinue | ForEach-Object Name) -join ", ")
}
