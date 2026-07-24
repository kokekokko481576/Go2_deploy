# scripts/ — セットアップ・計測スクリプト一覧

このディレクトリは、Go2_deploy プロジェクトの環境構築・動作確認・定量計測スクリプトをまとめています。

---

## 🚀 初回セットアップ（最初の1回だけ）

新しいマシンにリポジトリを構築するときは、以下を **上から順に** 実行してください。

### 1. `ubuntu-setup.sh` — ホスト側準備（Ubuntu）

```bash
curl -fsSL https://raw.githubusercontent.com/kokekokko481576/Go2_deploy/main/scripts/ubuntu-setup.sh | bash
```

**何をするのか:**
- gh CLI（GitHub CLI）を導入
- Docker Engine + compose plugin を導入
- `xhost +local:docker` を `~/.bashrc` / `~/.zshrc` に追記（コンテナからのGUI描画を許可）
- `/dev/dri`（GPU）の有無を確認

**対象環境:**
- ネイティブ Ubuntu（20.04 / 22.04 / 24.04 / 26.04 対応）
- 26.04 の場合は自動的に codename をフォールバック

**実行後:**
- ターミナルを一度開き直す（docker グループの反映に必要）

---

### 2. `ubuntu-clone.sh` — リポジトリ取得（Ubuntu）

```bash
curl -fsSL https://raw.githubusercontent.com/kokekokko481576/Go2_deploy/main/scripts/ubuntu-clone.sh | bash
```

**何をするのか:**
- GitHub CLI で `~/bridge/Go2_deploy/` に clone
- SSH で submodule（`external/*`）を取得
- `/dev/dri` 不在時に compose.override.yaml を自動生成

**gh 認証:**
- 初回実行時に `gh auth login` の対話あり
- ブラウザ認証が手軽（SSH 鍵は自動生成・登録）

**実行後:**
- chapter1/ が未取得のまま → 正常（submodule は update=none）

---

### 3. `first-run.sh` — 初回ビルド・起動・疎通チェック

```bash
cd ~/bridge/Go2_deploy && ./scripts/first-run.sh
```

**何をするのか:**
- docker/sim と docker/ の両イメージをビルド（初回は 15～40 分）
- 起動して DDS 疎通を自動チェック
- Gazebo 起動・ロボット登場・/robot1/* トピック確認

**期待される出力:**
- `[OK] simコンテナから go2が上がってきた`
- `[OK] devコンテナから /robot1/* トピックが見える`
- `[OK] 実データ(odometry)がdev側に届いている`

**2 回目以降:**
- キャッシュが効くので高速（ビルドスキップ）
- 起動のみ実行される

---

## 🧪 定量計測（Issue #36: GATE1）

GATE1（三者合流：自己位置推定 + 経路生成 + 経路追従）の定量ベースライン化のためのスクリプト。

### 準備

```bash
# Terminal 1: simコンテナ起動（Nav2干渉排除）
cd ~/bridge/Go2_deploy/docker/sim
SIM_ENABLE_NAV2=false docker compose up -d

# Terminal 2: devコンテナ起動
cd ~/bridge/Go2_deploy/docker
docker compose up -d

# Terminal 3: 自作EKF/AMCL + controller_server + planner起動
# 詳細: ros2_ws/src/go2_path_following/README.md「使い方(フェーズB)」参照
ros2 launch go2_localization localization.launch.py
ros2 launch go2_path_following controller.launch.py
ros2 run go2_path_following plan_follower
ros2 run straight_line_planner straight_line_planner_node --ros-args -p use_sim_time:=true -r /tf:=/go2_localization/tf
ros2 run cmd_vel_safety cmd_vel_safety_node --ros-args -r cmd_vel:=/robot1/cmd_vel
```

### `send_goal.sh` — ゴール投入

```bash
./scripts/send_goal.sh [x] [y] [yaw(度)]
```

**使い方:**

```bash
./scripts/send_goal.sh 3.0 0.0      # 3m前方へ、0°向き
./scripts/send_goal.sh 2.0 1.5 90   # (2, 1.5)へ、90°左向きに正対
```

**内部処理:**
- 度をラジアンに変換
- クォータニオン（z軸回転）を計算
- PoseStamped を `/goal_pose` に publish

**デフォルト:**
- 引数省略時は `3.0 0.0 0°` でゴール投入

---

### `gate1_measure.sh` — 計測自動化

```bash
./scripts/gate1_measure.sh [trials]
```

**使い方:**

```bash
./scripts/gate1_measure.sh 50        # 50 試行
./scripts/gate1_measure.sh           # デフォルト50試行
```

**実施内容:**
1. ランダムゴール生成（x: 1～3m, y: -1～1m, yaw: 0～360°）
2. ゴール投入（`send_goal.sh` 呼び出し）
3. 到達待機（12 秒）
4. 最終姿勢・タイムスタンプをログに記録
5. 繰り返す

**出力:**
```
=== GATE1 Quantification: 50 trials ===
Results directory: /tmp/gate1_results_20260724_151200
[  1/50] Goal=( 1.50,  0.30, 120.0°) LOGGED
[  2/50] Goal=( 2.80, -0.60,  45.3°) LOGGED
...
=== Summary ===
Results saved to: /tmp/gate1_results_20260724_151200
Logs: 50 files
```

**ログフォーマット:**
```
trial: 1
goal: {x: 1.50, y: 0.30, yaw: 120.0}
timestamp: 2026-07-24T15:12:00+09:00
```

---

### `gate1_analyze.py` — 統計解析

```bash
./scripts/gate1_analyze.py <results_directory>
```

**使い方:**

```bash
./scripts/gate1_analyze.py /tmp/gate1_results_20260724_151200
```

**出力:**
- ログファイル数（試行数）
- サンプル 3 件の内容表示
- レポート生成

**今後の拡張（詳細計測対応時）:**
- xy 誤差の分布（平均・標準偏差）
- yaw 誤差の分布
- 成功率計算
- CSV 出力
- グラフ生成

---

## 💻 Windows 向けセットアップ

### `windows-setup.ps1` — Windows 準備

```powershell
irm https://raw.githubusercontent.com/kokekokko481576/Go2_deploy/main/scripts/windows-setup.ps1 | iex
```

**対象:** Windows 10 / 11 + WSL2

**何をするのか:**
- WSL2 導入（初回）
- Ubuntu イメージダウンロード
- GPU ドライバチェック（AMD/NVIDIA）

**実行後:**
- 再起動 → Ubuntu ユーザ作成画面へ

---

### `wsl2-setup.sh` — WSL2 内セットアップ

```bash
curl -fsSL https://raw.githubusercontent.com/kokekokko481576/Go2_deploy/main/scripts/wsl2-setup.sh | bash
```

**対象:** WSL2 内の Ubuntu

**何をするのか:**
- Ubuntu 内に gh CLI・Docker Engine を導入
- xhost ネイティブ WSL2 経由の設定

**実行後:**
- ターミナル開き直し → `cd ~/Go2_deploy && ./scripts/first-run.sh`

---

## 📋 スクリプトの使い分け

| 環境 | 最初（1 回目） | 再起動後 | 日常実行 |
|------|-------------|---------|--------|
| **ネイティブ Ubuntu** | ① ubuntu-setup.sh<br>② ubuntu-clone.sh<br>③ first-run.sh | ③ first-run.sh | — |
| **Windows WSL2** | windows-setup.ps1<br>（Windows 側）<br>wsl2-setup.sh<br>（WSL2 側）<br>first-run.sh | first-run.sh | — |
| **計測（GATE1）** | 上記セットアップ完了後 | — | gate1_measure.sh<br>gate1_analyze.py<br>send_goal.sh |

---

## 🔧 トラブルシュート

### Q: `docker compose up` が `error gathering device` で失敗する

**A:** `/dev/dri` が無い環境。`compose.override.yaml.example` を `compose.override.yaml` にコピーして GPU 無効化。

```bash
cp docker/compose.override.yaml.example docker/compose.override.yaml
cp docker/sim/compose.override.yaml.example docker/sim/compose.override.yaml
```

### Q: `ubuntu-setup.sh` 後も docker が使えない

**A:** ターミナルを開き直す（docker グループ反映）。即席で試すなら `newgrp docker`。

### Q: Gazebo が真っ黒 / 激重

**A:** GPU ソフトレンダリング（mesa）。動作はするので先へ進んでよい。詳細は `docs/手順/Windows-WSL2セットアップ.md` トラブルシュート参照。

### Q: gh 認証エラー

**A:** `gh auth logout` → `ubuntu-setup.sh` と `ubuntu-clone.sh` を再実行。SSH 鍵が GitHub に登録されているか確認。

---

## 📚 関連ドキュメント

- `docs/手順/Ubuntuセットアップ.md` — 詳細手順・トラブルシュート
- `docs/手順/Windows-WSL2セットアップ.md` — Windows 向け詳細
- `docs/手順/デュアルブート構築.md` — Ubuntu デュアルブート構築
- `ONBOARDING.md` — 快速スタートガイド

---

**最後更新:** 2026-07-24
**関連 Issue:** #28/#29/#30/#36/#40
