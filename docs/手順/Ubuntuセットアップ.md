# ネイティブ Ubuntu セットアップ (Issue #40)

Ubuntu を入れ終えた直後から、このリポジトリの Docker 環境を動かすまでの手順。
コンテナ中身は `ros:humble`（Ubuntu 22.04 固定）なので、**ROS2 の動作はホストの
Ubuntu バージョンに依存しない**（20.04 でも 24.04 でも 26.04 でも同じ）。ホスト依存が
出るのは GUI（X11/Wayland）と GPU（`/dev/dri`）だけで、それをスクリプトで整える。

> Windows の場合は代わりに `Windows-WSL2セットアップ.md` を参照。

## 最短手順（スクリプト2本、これをやるだけ）

1. **ホスト側の準備**（gh CLI・Docker Engine 導入、`xhost` を rc に追記）

   ```bash
   curl -fsSL https://raw.githubusercontent.com/kokekokko481576/Go2_deploy/main/scripts/ubuntu-setup.sh | bash
   ```

   終わったら **ターミナルを一度開き直す**（docker グループの反映に必要）。

2. **リポジトリ取得 + 固有設定**（`~/bridge/Go2_deploy` へ clone）

   ```bash
   curl -fsSL https://raw.githubusercontent.com/kokekokko481576/Go2_deploy/main/scripts/ubuntu-clone.sh | bash
   ```

   初回は `gh auth login` の対話が入る（ブラウザ認証が手軽）。

3. **初回ビルド + 起動 + 疎通チェック**

   ```bash
   cd ~/bridge/Go2_deploy && ./scripts/first-run.sh
   ```

以降の節は、スクリプトが中でやっていることの説明と、詰まったとき用の詳細。

### 何度実行しても安全（冪等性）

3本とも導入済み・設定済みは自動スキップし、既存環境を壊さない設計。実際に
gh・Docker 導入済みの機で 2 回連続実行してもエラーなし・二重追記なしを確認済み。

- `ubuntu-setup.sh`: gh/Docker 導入済みならスキップ、docker グループ所属済みなら
  そのまま、xhost 追記はマーカーで二重追記しない。
- `ubuntu-clone.sh`: `~/bridge/Go2_deploy` が既にあれば **clone せず作業ツリーに一切
  触れない**。submodule は「未取得のときだけ」init し、取得済みなら手を触れない
  （チェックアウト状態を勝手に動かさない）。

## 0. gh の認証（ブラウザ認証・全手順）

`ubuntu-clone.sh` は clone に gh を使うため、初回は 1 度だけ認証が要る（`ubuntu-setup.sh`
で gh を入れた後）。スクリプトは未認証を検知すると自動で `gh auth login` を起動するが、
中の対話は次のとおり答える。手動で先に済ませておいてもよい。

```bash
gh auth login
```

対話の答え方（↑↓で選び Enter）:

1. **What account do you want to log into?** → `GitHub.com`
2. **What is your preferred protocol for Git operations?** → `SSH`
   （このリポジトリと submodule はネイティブの `git@github.com:` を使う）
3. **Generate a new SSH key to add to your GitHub account?** → `Yes`
   （既に鍵があれば既存鍵を選んでもよい。パスフレーズは任意）
   - gh が鍵を生成し、**GitHub アカウントへ自動でアップロード**してくれる。
     鍵タイトルを聞かれたら任意の名前でよい。
4. **How would you like to authenticate GitHub CLI?** → `Login with a web browser`
5. 画面に **one-time code**（例 `ABCD-1234`）が出る → メモして Enter
6. ブラウザが開く（開かなければ表示された URL を手で開く）→ GitHub にログイン →
   コードを入力 → **Authorize** を押す
7. ターミナルに `✓ Logged in as <ユーザ名>` が出れば完了

> SSH を選ぶ理由: チームの Git 運用を SSH で統一するため。gh が鍵の生成〜登録まで
> やってくれるので、新規マシンでも手作業の鍵登録は不要。

確認・やり直し:

```bash
gh auth status     # 認証状態の確認
gh auth logout     # やり直したいとき
```

## 1. 事前チェック（ubuntu-setup.sh がやること）

- **セッション種別**（`$XDG_SESSION_TYPE`）: `wayland` だと GUI（RViz2/Gazebo）が
  出ないことがある。**26.04 は Wayland 既定**の見込みなので、GUI を使うなら
  ログイン画面の歯車から「Ubuntu on Xorg」を選び直す。
- **`/dev/dri`**: コンテナへ渡す iGPU。無くても `docker compose build` は通り、
  `up` 段階でだけ効く。多くは Ubuntu 標準の mesa で自動認識される。

## 2. gh CLI と Docker Engine（ubuntu-setup.sh がやること）

- gh は公式 apt リポジトリ（`cli.github.com`）から導入。
- Docker は公式 apt リポジトリ（`download.docker.com`）から `docker-ce` +
  `docker-compose-plugin` を導入。
  - **26.04 対応**: Docker リポジトリは新リリース直後だと当該 codename の
    ディレクトリが未整備で 404 することがある。スクリプトは
    「実 codename → `noble` → `jammy`」の順に実在するものを自動採用する
    （別 codename 版で代用しても Docker の動作に問題はない）。
- `usermod -aG docker $USER` を実行。**反映は再ログイン後**。
  即席で試すなら `newgrp docker`。

## 3. xhost の rc 追記（ubuntu-setup.sh がやること）

RViz2/Gazebo のコンテナからホストの X11 へ描画するため、`~/.bashrc` と
`~/.zshrc` に次を冪等追記する（マーカーで二重追記を防止）。GUI セッション
（`DISPLAY` あり）のときだけ実行され、CUI ログインの邪魔をしない。

```bash
command -v xhost >/dev/null 2>&1 && [ -n "${DISPLAY:-}" ] && xhost +local:docker >/dev/null 2>&1
```

## 4. リポジトリ取得と固有設定（ubuntu-clone.sh がやること）

- `gh auth status` で認証を確認（未認証なら「0. gh の認証」の対話を自動起動）。
- `~/bridge/Go2_deploy` が無ければ `gh repo clone` で取得。**既にあれば clone せず、
  作業ツリーに一切触れない**（自分の変更を消さない）。
- submodule: `chapter1` は `update=none`（未取得で正常）。`external/*` はネイティブの
  `git@github.com:`（SSH）でそのまま取得（gh auth login で登録済みの鍵を使う）。取得は
  **新規 clone 時か、external が未取得のときだけ**行い、取得済みなら手を触れない。
- `/dev/dri` が無い機では、`compose.override.yaml.example` を
  `compose.override.yaml` にコピーして GPU 渡しを無効化する（既にあれば触らない）。

## トラブルシュート

- **`docker compose up` が `/dev/dri` で失敗**（`error gathering device`）:
  GPU 渡しを無効化する。`ubuntu-clone.sh` は `/dev/dri` 不在を検知して自動で
  override を用意するが、手動でやる場合は次を実行。

  ```bash
  cp docker/compose.override.yaml.example docker/compose.override.yaml
  cp docker/sim/compose.override.yaml.example docker/sim/compose.override.yaml
  ```

- **GUI（RViz2/Gazebo）が出ない**: `echo $XDG_SESSION_TYPE` が `wayland` なら
  Xorg でログインし直す。`xhost` の追記はターミナル再起動後に効く。

- **`docker` が使えない**: `ubuntu-setup.sh` 後にターミナルを開き直したか確認。
  即席なら `newgrp docker`。デーモン未起動なら
  `sudo systemctl enable --now docker`。

- **GPU ドライバ**: NVIDIA/Isaac 系（#33）は本手順の対象外。iGPU は Ubuntu 標準
  の mesa で概ね連携済みの想定で、ドライバ導入はスクリプトでは行わない。
