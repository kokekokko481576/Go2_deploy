#!/usr/bin/env bash
# Go2_deploy: gh CLIでのリポジトリ取得 + 固有設定 (Issue #40)
# ubuntu-setup.sh の後、ターミナルを開き直してから:
#   curl -fsSL https://raw.githubusercontent.com/kokekokko481576/Go2_deploy/main/scripts/ubuntu-clone.sh | bash
# やること: gh認証確認 → ~/bridge/Go2_deploy へ clone → 固有設定(/dev/dri不在時のcompose.override)。
# 何度実行しても安全(取得済み・設定済みはスキップ)。

set -u
FAIL=0
ok() { echo "  [OK] $1"; }
ng() { echo "  [NG] $1"; FAIL=1; }
warn() { echo "  [注意] $1"; }

REPO="kokekokko481576/Go2_deploy"
TARGET="$HOME/bridge/Go2_deploy"

echo "=== 1/3 前提コマンド ==="
command -v gh >/dev/null 2>&1 && ok "gh あり" || { ng "gh が無い → 先に ubuntu-setup.sh を実行"; exit 1; }
command -v git >/dev/null 2>&1 && ok "git あり" || { ng "git が無い → sudo apt-get install -y git"; exit 1; }

echo "=== 2/3 GitHub認証 & clone ==="
if gh auth status >/dev/null 2>&1; then
    ok "gh 認証済み"
else
    warn "gh 未認証 → 対話ログインを開始する(ブラウザ or トークン)"
    gh auth login || { ng "gh auth login に失敗"; exit 1; }
fi

FRESH_CLONE=0
if [ -d "$TARGET/.git" ]; then
    ok "$TARGET は取得済み(既存の作業ツリーには手を触れない)"
else
    mkdir -p "$(dirname "$TARGET")"
    # gh でリポジトリ本体を取得(submoduleは後段で明示的に読み替えて取得する)
    if gh repo clone "$REPO" "$TARGET" >/dev/null 2>&1; then
        ok "$TARGET に clone した"
        FRESH_CLONE=1
    else
        ng "clone に失敗(認証・ネットワークを確認)"
        exit 1
    fi
fi

# submodule: chapter1 は update=none(取得されなくて正常)。external/* をSSHで取得する。
# gh auth login(SSH)でGitHubに鍵が登録済みである前提(URLはネイティブのgit@github.com:のまま)。
# 既存repoのときは作業ツリーを尊重し、submoduleが「未取得のときだけ」initする
# (取得済みの submodule には update をかけず、チェックアウト状態を勝手に動かさない)。
sm_populated() { [ -n "$(ls -A "$TARGET/external/go2_ros2_sim_py" 2>/dev/null)" ]; }
if [ "$FRESH_CLONE" -eq 1 ]; then
    git -C "$TARGET" submodule update --init --recursive external >/dev/null 2>&1 &&
        ok "submodule(external/*)を取得した(chapter1は未取得で正常)" ||
        warn "submodule取得に一部失敗。SSH鍵が未登録なら gh auth login をやり直す。simを使わないなら無視可(dev環境のbuildには不要)"
elif sm_populated; then
    ok "submodule(external/*)は取得済み → 触らない"
else
    warn "既存repoだが external/* が未取得 → init のみ実行(既存の他ファイルは触らない)"
    git -C "$TARGET" submodule update --init --recursive external >/dev/null 2>&1 &&
        ok "submodule(external/*)を取得した" ||
        warn "submodule取得に一部失敗。SSH鍵が未登録なら gh auth login をやり直す。simを使わないなら無視可"
fi

echo "=== 3/3 固有設定 ==="
# /dev/dri が無い機ではcomposeのdevices指定でupが失敗するため、雛形からoverrideを用意して無効化する。
if [ -e /dev/dri/renderD128 ] || [ -e /dev/dri/card0 ]; then
    ok "/dev/dri あり → override不要(GPU渡しが有効)"
else
    warn "/dev/dri なし → GPU渡しを無効化するoverrideを用意する"
    for d in docker docker/sim; do
        ex="$TARGET/$d/compose.override.yaml.example"
        ov="$TARGET/$d/compose.override.yaml"
        if [ -f "$ov" ]; then
            ok "$d/compose.override.yaml は作成済み"
        elif [ -f "$ex" ]; then
            cp "$ex" "$ov" && ok "$d/compose.override.yaml を雛形から作成"
        fi
    done
fi

echo ""
if [ "$FAIL" -eq 0 ]; then
    echo "=== 取得完了 ==="
    echo "次: 初回ビルド+起動+疎通チェック"
    echo "  cd $TARGET && ./scripts/first-run.sh"
    echo "(dockerグループ未反映で失敗する場合は、ターミナルを開き直すか newgrp docker してから)"
else
    echo "=== NGあり。上のメッセージに従って解消してから再実行してください ==="
fi
exit "$FAIL"
