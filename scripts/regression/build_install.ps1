# Fast incremental build + install of the JAM-related pieces.
#
#   .\build_install.ps1 -Prefix C:\...\EMG-Assisted-COMAK-install-step1 `
#                       -SeedFrom C:\...\EMG-Assisted-COMAK-install-rwdi
#
# - builds only the targets that contain JAM code (not apps/examples/docs)
# - uses up to 12 parallel jobs
# - seeds a new install folder from a previous install with robocopy, so
#   `cmake --install` only has to refresh files that changed
param(
    [Parameter(Mandatory = $true)][string]$Prefix,
    [string]$SeedFrom = "",
    [string]$BuildDir = "C:\Users\amires\Desktop\GitHub\EMG_COMAK\EMG_COMAK_BLD",
    [string]$Config = "RelWithDebInfo",
    [int]$Jobs = 12,
    [string[]]$Targets = @("osimJAM", "_jam", "JavaBindings")
)

$ErrorActionPreference = "Stop"
$sw = [Diagnostics.Stopwatch]::StartNew()

$targetArgs = @()
foreach ($t in $Targets) { $targetArgs += @("--target", $t) }
& cmake --build $BuildDir --config $Config --parallel $Jobs @targetArgs
if ($LASTEXITCODE -ne 0) { throw "build failed" }
"build done in $([int]$sw.Elapsed.TotalSeconds) s"

if ($SeedFrom -and -not (Test-Path $Prefix)) {
    # exit codes 0-7 mean success for robocopy
    robocopy $SeedFrom $Prefix /E /NFL /NDL /NJH /NJS /NP /MT:12 | Out-Null
    if ($LASTEXITCODE -gt 7) { throw "robocopy failed ($LASTEXITCODE)" }
    "seeded $Prefix from $SeedFrom in $([int]$sw.Elapsed.TotalSeconds) s"
}

& cmake --install $BuildDir --config $Config --prefix $Prefix | Select-Object -Last 3
if ($LASTEXITCODE -ne 0) { throw "install failed" }
"all done in $([int]$sw.Elapsed.TotalSeconds) s"
