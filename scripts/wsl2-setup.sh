#!/usr/bin/env bash
# Go2_deploy: WSL2のUbuntu内セットアップ (Issue #28)
# Ubuntu(WSL2)のターミナルで:
#   curl -fsSL https://raw.githubusercontent.com/kokekokko481576/Go2_deploy/main/scripts/wsl2-setup.sh | bash
# やること: 事前チェック(GPU/GUI/WSL2) → Docker Engine導入 → リポジトリclone。
# 何度実行しても安全(導入済みの項目はスキップ)。

set -u
FAIL=0
ok() { echo "  [OK] $1"; }
ng() { echo "  [NG] $1"; FAIL=1; }
warn() { echo "  [注意] $1"; }

echo "=== 1/4 事前チェック ==="
grep -qi microsoft /proc/version && ok "WSL2カーネル" || ng "WSL2ではない環境に見える(このスクリプトはWSL2用)"
[ -e /dev/dxg ] && ok "/dev/dxg (GPU仮想化)" || ng "/dev/dxg なし → Windows側GPUドライバを更新し、PowerShellで wsl --shutdown 後にやり直し"
if [ -e /dev/dri/renderD128 ] || [ -e /dev/dri/card0 ]; then
    ok "/dev/dri (コンテナへのGPU渡しがそのまま使える)"
else
    warn "/dev/dri なし → Gazeboはソフトレンダリングになる(遅いが動く)。docs/手順/Windows-WSL2セットアップ.md のトラブルシュート参照"
fi
[ -S /tmp/.X11-unix/X0 ] && ok "WSLg X11ソケット" || ng "X11ソケットなし → PowerShellで wsl --update && wsl --shutdown 後にやり直し"
[ -n "${DISPLAY:-}" ] && ok "DISPLAY=${DISPLAY}" || ng "DISPLAYが空 → WSLgが動いていない"

echo "=== 2/4 Docker Engine ==="
if command -v docker >/dev/null 2>&1; then
    ok "docker は導入済み"
else
    echo "  Docker Engine(docker-ce)を導入します(sudoパスワードを聞かれます)"
    sudo apt-get update -qq
    sudo apt-get install -y -qq ca-certificates curl >/dev/null
    sudo install -m 0755 -d /etc/apt/keyrings
    sudo curl -fsSL https://download.docker.com/linux/ubuntu/gpg -o /etc/apt/keyrings/docker.asc
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.asc] \
https://download.docker.com/linux/ubuntu $(. /etc/os-release && echo "$VERSION_CODENAME") stable" |
        sudo tee /etc/apt/sources.list.d/docker.list >/dev/null
    sudo apt-get update -qq
    sudo apt-get install -y -qq docker-ce docker-ce-cli containerd.io docker-compose-plugin >/dev/null
    ok "docker を導入した"
fi
sudo usermod -aG docker "$USER"
if systemctl is-active --quiet docker 2>/dev/null; then
    ok "dockerデーモン稼働中"
else
    ng "dockerデーモンが起動していない → systemd無効の可能性。/etc/wsl.conf に [boot]systemd=true を書き、PowerShellで wsl --shutdown"
fi

echo "=== 3/4 リポジトリ取得 ==="
if [ -d "$HOME/Go2_deploy/.git" ]; then
    ok "~/Go2_deploy は取得済み"
else
    # submoduleがSSH URL参照のため、このclone操作に限りHTTPSへ読み替える
    # (--global設定は汚さない。SSH鍵運用の人にも影響しない)
    git -c url."https://github.com/".insteadOf="git@github.com:" \
        clone --recurse-submodules https://github.com/kokekokko481576/Go2_deploy.git "$HOME/Go2_deploy" &&
        ok "~/Go2_deploy にcloneした(chapter1は未取得で正常)" || ng "cloneに失敗"
fi
case "$HOME" in /mnt/*) ng "ホームが /mnt 配下?? cloneはWSL2側FSに置くこと(#32)";; *) ok "clone先はWSL2側FS";; esac

echo "=== 4/4 結果 ==="
if [ "$FAIL" -eq 0 ]; then
    echo "セットアップ完了。**ターミナルを一度開き直してから**(dockerグループ反映) 次を実行:"
    echo "  cd ~/Go2_deploy && ./scripts/first-run.sh"
else
    echo "NG項目があります。上のメッセージに従って解消してから再実行してください"
    echo "(何度実行しても安全です)"
fi
exit "$FAIL"
