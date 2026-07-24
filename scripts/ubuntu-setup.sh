#!/usr/bin/env bash
# Go2_deploy: ネイティブUbuntuのホスト側セットアップ (Issue #40)
# Ubuntuを入れ終えた直後のターミナルで:
#   curl -fsSL https://raw.githubusercontent.com/kokekokko481576/Go2_deploy/main/scripts/ubuntu-setup.sh | bash
# やること: 事前チェック → gh CLI導入 → Docker Engine導入 → xhostをrcに追記。
# 何度実行しても安全(導入済みの項目はスキップ)。26.04でもcodenameフォールバックで動く。
# コンテナ中身はros:humble(Ubuntu22.04固定)なのでROS2動作はホストのUbuntu版に依存しない。
# ホスト依存はGUI(X11/Wayland)とGPU(/dev/dri)だけで、それをここで整える。

set -u
FAIL=0
ok() { echo "  [OK] $1"; }
ng() { echo "  [NG] $1"; FAIL=1; }
warn() { echo "  [注意] $1"; }

echo "=== 1/4 事前チェック ==="
if grep -qi microsoft /proc/version 2>/dev/null; then
    warn "WSL2環境に見える。WSL2なら scripts/wsl2-setup.sh の方を使うこと(このスクリプトはネイティブUbuntu用)"
fi
. /etc/os-release 2>/dev/null || true
ok "ホスト: ${PRETTY_NAME:-Ubuntu(不明)}"

# GUI(RViz2/Gazebo)はホスト側のX11に描画する。Waylandだとxhost/X11が素直に通らないことがある。
case "${XDG_SESSION_TYPE:-}" in
    x11) ok "セッションはX11 (RViz2/GazeboのGUIがそのまま通る)" ;;
    wayland) warn "セッションがWayland。GUIが出ない場合はログイン画面で歯車→「Ubuntu on Xorg」を選び直す(26.04はWayland既定)" ;;
    *) warn "セッション種別が不明(${XDG_SESSION_TYPE:-空})。CUIのみ運用なら無視可。GUIを使うならX11ログイン推奨" ;;
esac

# コンテナへ渡すiGPU。無くてもビルドは可、up段階でだけ効く(compose.override.yaml.exampleで回避可)。
if [ -e /dev/dri/renderD128 ] || [ -e /dev/dri/card0 ]; then
    ok "/dev/dri (iGPU: コンテナへのGPU渡しがそのまま使える)"
else
    warn "/dev/dri なし → docker compose up が失敗しうる。多くはUbuntu標準のmesaで自動認識されるが、出ない場合は clone後に docker/compose.override.yaml.example を参照"
fi

echo "=== 2/4 GitHub CLI (gh) ==="
if command -v gh >/dev/null 2>&1; then
    ok "gh は導入済み"
else
    echo "  gh を導入します(sudoパスワードを聞かれます)"
    sudo apt-get update -qq
    sudo apt-get install -y -qq ca-certificates curl >/dev/null
    sudo install -m 0755 -d /etc/apt/keyrings
    sudo curl -fsSL https://cli.github.com/packages/githubcli-archive-keyring.gpg \
        -o /etc/apt/keyrings/githubcli-archive-keyring.gpg
    sudo chmod go+r /etc/apt/keyrings/githubcli-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/githubcli-archive-keyring.gpg] \
https://cli.github.com/packages stable main" |
        sudo tee /etc/apt/sources.list.d/github-cli.list >/dev/null
    sudo apt-get update -qq
    sudo apt-get install -y -qq gh >/dev/null && ok "gh を導入した" || ng "gh の導入に失敗"
fi

echo "=== 3/4 Docker Engine ==="
if command -v docker >/dev/null 2>&1; then
    ok "docker は導入済み"
else
    echo "  Docker Engine(docker-ce)を導入します"
    # Dockerリポジトリに該当codenameが未整備(新リリース直後の26.04など)だと404するため、
    # 実codename → 既知の安定codenameの順で、実在するものを採用する。
    DOCKER_CODENAME=""
    for c in "$(. /etc/os-release && echo "$VERSION_CODENAME")" noble jammy; do
        [ -n "$c" ] || continue
        if curl -fsSL "https://download.docker.com/linux/ubuntu/dists/$c/Release" -o /dev/null 2>/dev/null; then
            DOCKER_CODENAME="$c"
            break
        fi
    done
    if [ -z "$DOCKER_CODENAME" ]; then
        ng "Dockerリポジトリで使えるcodenameが見つからない(ネットワーク不通?)"
    else
        [ "$DOCKER_CODENAME" = "$(. /etc/os-release && echo "$VERSION_CODENAME")" ] ||
            warn "Dockerリポジトリに ${VERSION_CODENAME} が無いため $DOCKER_CODENAME 版で代用(動作は問題ない)"
        sudo apt-get update -qq
        sudo apt-get install -y -qq ca-certificates curl >/dev/null
        sudo install -m 0755 -d /etc/apt/keyrings
        sudo curl -fsSL https://download.docker.com/linux/ubuntu/gpg -o /etc/apt/keyrings/docker.asc
        sudo chmod a+r /etc/apt/keyrings/docker.asc
        echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.asc] \
https://download.docker.com/linux/ubuntu $DOCKER_CODENAME stable" |
            sudo tee /etc/apt/sources.list.d/docker.list >/dev/null
        sudo apt-get update -qq
        sudo apt-get install -y -qq docker-ce docker-ce-cli containerd.io docker-compose-plugin >/dev/null &&
            ok "docker を導入した" || ng "docker の導入に失敗"
    fi
fi
if id -nG "$USER" 2>/dev/null | tr ' ' '\n' | grep -qx docker; then
    ok "$USER は既に docker グループ所属"
else
    sudo usermod -aG docker "$USER" && ok "$USER を docker グループに追加(反映は再ログイン後)"
fi
if systemctl is-active --quiet docker 2>/dev/null; then
    ok "dockerデーモン稼働中"
else
    warn "dockerデーモンが未稼働 → sudo systemctl enable --now docker を試す"
fi

echo "=== 4/4 xhost を rc に追記(コンテナのGUI許可) ==="
# RViz2/GazeboのコンテナからホストのX11へ描画するため、シェル起動時に許可を出す。
# GUIセッション(DISPLAYあり)のときだけ実行する形にして、CUIログインの邪魔をしない。
XHOST_MARKER="# >>> Go2_deploy xhost (docker GUI) >>>"
append_xhost() {
    rc="$1"
    if [ -f "$rc" ] && grep -qF "$XHOST_MARKER" "$rc"; then
        ok "$(basename "$rc") は追記済み"
        return 0
    fi
    cat >>"$rc" <<EOF

$XHOST_MARKER
# コンテナ(RViz2/Gazebo)のX11接続を許可。GUIセッション時のみ実行。
command -v xhost >/dev/null 2>&1 && [ -n "\${DISPLAY:-}" ] && xhost +local:docker >/dev/null 2>&1
# <<< Go2_deploy xhost (docker GUI) <<<
EOF
    ok "$(basename "$rc") に xhost を追記した"
}
append_xhost "$HOME/.bashrc"
append_xhost "$HOME/.zshrc"

echo "=== 結果 ==="
if [ "$FAIL" -eq 0 ]; then
    echo "ホスト側の準備完了。**ターミナルを一度開き直してから**(dockerグループ反映) 次を実行:"
    echo "  curl -fsSL https://raw.githubusercontent.com/kokekokko481576/Go2_deploy/main/scripts/ubuntu-clone.sh | bash"
else
    echo "NG項目があります。上のメッセージに従って解消してから再実行してください(何度実行しても安全)。"
fi
exit "$FAIL"
