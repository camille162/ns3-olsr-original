#Requires -Version 5.1
<#
.SYNOPSIS
  Initialize (if needed), commit, and push this project to GitHub.

.DESCRIPTION
  Repository: https://github.com/camille162/ns3-olsr-original

  Run from anywhere:
    powershell -ExecutionPolicy Bypass -File scripts/push-to-github.ps1

  Prerequisites:
  - Git for Windows installed (https://git-scm.com/download/win)
  - GitHub authentication: HTTPS (PAT or credential manager) or SSH remote.

  If the remote already has commits you do not have locally, use:
    git pull origin main --rebase
  before pushing, or resolve conflicts as prompted.

.NOTES
  Repo root = directory containing README.md (parent of scripts/).
#>

$ErrorActionPreference = "Stop"

$repoRoot = Split-Path -Parent $PSScriptRoot
Set-Location $repoRoot

$git = Get-Command git -ErrorAction SilentlyContinue
if (-not $git) {
    $candidates = @(
        "${env:ProgramFiles}\Git\cmd\git.exe",
        "${env:ProgramFiles}\Git\bin\git.exe",
        "${env:ProgramFiles(x86)}\Git\cmd\git.exe"
    )
    foreach ($p in $candidates) {
        if (Test-Path $p) {
            $git = $p
            break
        }
    }
}

if (-not $git) {
    Write-Host "Git not found. Install Git for Windows, then open a new terminal and run this script again." -ForegroundColor Red
    exit 1
}

$gitExe = if ($git -is [string]) { $git } else { $git.Source }

Write-Host "Repo root: $repoRoot" -ForegroundColor Cyan
Write-Host "Using git: $gitExe" -ForegroundColor Cyan

if (-not (Test-Path (Join-Path $repoRoot ".git"))) {
    & $gitExe init
}

$originUrl = "https://github.com/camille162/ns3-olsr-original.git"
$remotes = @(& $gitExe remote 2>$null)
if ($remotes -contains "origin") {
    & $gitExe remote set-url origin $originUrl
} else {
    & $gitExe remote add origin $originUrl
}

& $gitExe add -A
$status = & $gitExe status --porcelain
if (-not $status) {
    Write-Host "Nothing to commit (working tree clean)." -ForegroundColor Yellow
} else {
    & $gitExe commit -m "Sync etx-olsr contrib layout and sources"
}

& $gitExe branch -M main
Write-Host "Pushing to origin main ..." -ForegroundColor Cyan
& $gitExe push -u origin main
