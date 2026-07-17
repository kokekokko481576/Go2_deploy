# Go2_deploy: Windows側セットアップ (Issue #28)
# 管理者PowerShellで:
#   irm https://raw.githubusercontent.com/kokekokko481576/Go2_deploy/main/scripts/windows-setup.ps1 | iex
# やること: WSL2+Ubuntu-22.04の導入(または更新)、.wslconfigの作成。
# GPUドライバの更新だけは自動化できないので最後に案内を出す。

$ErrorActionPreference = "Stop"
Write-Host "=== Go2_deploy Windows(WSL2) セットアップ ===" -ForegroundColor Cyan

# 管理者チェック
$isAdmin = ([Security.Principal.WindowsPrincipal][Security.Principal.WindowsIdentity]::GetCurrent()).IsInRole([Security.Principal.WindowsBuiltInRole]::Administrator)
if (-not $isAdmin) {
    Write-Host "管理者PowerShellで実行してください(スタートメニュー→PowerShell右クリック→管理者として実行)" -ForegroundColor Red
    exit 1
}

# WSLの有無で導入/更新を分岐
$wslInstalled = $true
try { wsl --status *> $null } catch { $wslInstalled = $false }
if (-not $wslInstalled -or $LASTEXITCODE -ne 0) {
    Write-Host "WSLを新規導入します(終わったら再起動が必要です)"
    wsl --install -d Ubuntu-22.04
    $needReboot = $true
} else {
    Write-Host "WSLは導入済み。更新します"
    wsl --update
    $distros = (wsl -l -q) -replace "`0", ""   # UTF-16のNUL除去
    if ($distros -notmatch "Ubuntu-22.04") {
        Write-Host "Ubuntu-22.04 を追加導入します"
        wsl --install -d Ubuntu-22.04
    }
    $needReboot = $false
}

# .wslconfig (メモリ上限。既存ファイルには触らない)
$cfg = "$env:UserProfile\.wslconfig"
if (-not (Test-Path $cfg)) {
    "[wsl2]`nmemory=12GB`n" | Out-File -Encoding ascii $cfg
    Write-Host ".wslconfig を作成しました (memory=12GB)"
} else {
    Write-Host ".wslconfig は既にあるので触りません(メモリ上限12GB程度を推奨)"
}

Write-Host ""
Write-Host "=== 残りの手順 ===" -ForegroundColor Cyan
Write-Host "1. GPUドライバをベンダー公式から最新に更新(AMDならAdrenalin) ※手動"
if ($needReboot) {
    Write-Host "2. Windowsを再起動 → Ubuntuが自動で開くので初期ユーザを作成"
    Write-Host "3. Ubuntuのターミナルで次を実行:"
} else {
    Write-Host "2. Ubuntu-22.04 を開いて(初回なら初期ユーザ作成)、次を実行:"
}
Write-Host '   curl -fsSL https://raw.githubusercontent.com/kokekokko481576/Go2_deploy/main/scripts/wsl2-setup.sh | bash' -ForegroundColor Yellow
