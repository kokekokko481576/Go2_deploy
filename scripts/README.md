# scripts/ — セットアップ・起動・計測スクリプト

Go2_deploy の環境構築・日常起動・定量計測をまとめたスクリプト集。

> スクリプトは2種類。**ホスト側**（repo直下 `scripts/`：docker を外から操作）と、
> **dev コンテナ内**（`ros2_ws/src/go2_path_following/scripts/`：`ros2` を直接呼ぶもの）。

## まず最初に：どれを使う？

| やりたいこと | 使うもの |
|---|---|
| **毎日の開発を始める**（docker起動〜自作スタック起動を一発） | [`dev-up.sh`](#dev-upsh--日常の起動-まずこれ) |
| **新しいマシンに初めて構築する** | [初回セットアップ](#初回セットアップ新しいマシンで1回だけ) |
| **GATE1 の精度を計測する** | [`gate1_measure.sh` / `gate1_analyze.py`](#gate1-定量計測) |
| **ゴールを1回投げたいだけ** | [`send_goal.sh`](#ゴール投入-send_goalsh) |

---

## `dev-up.sh` — 日常の起動（まずこれ）

```bash
cd ~/bridge/Go2_deploy && ./scripts/dev-up.sh
```

これ1本で **docker(sim/dev)起動 → ロボット起動待ち → `colcon build` → 自作スタックの起動** まで進む。
起動後は「**自分が新規開発したノードを別ターミナルで動かすだけ**」の状態になる。

### 何を起動するか対話で選ぶ

実行すると3つ聞かれる。**自作サブシステムの中身を知らなくてもブラックボックスとして起動できる**のが狙い。

```
[1] 自己位置推定 (/go2_localization/tf の供給元)
    1) 実装済み推定 (EKF/AMCL)  ← 既定
    2) 自作 (起動しない。自分で /go2_localization/tf を出す)
[2] 経路生成の見本(straight_line_planner)を起動する? [Y/n]
[3] 経路追従(controller_server)を起動する? [Y/n]
```

- **自己位置推定を自作する人** → [1] で `2)` を選び、自分の推定が `/go2_localization/tf` を出す。
- **経路生成が本命の人** → [2] を `n`。自分のプランナを別ターミナルで起動（起動後にコマンド例が出る）。
- **経路追従(controller)を改良する人** → [3] を `Y`（or 自分の複製を使うなら `n`）。

`plan_follower`（/plan → 追従命令の橋渡し）と `cmd_vel_safety`（速度の安全弁）は**常時起動**。

### 起動後の流れ

`dev-up.sh` はログを流しながら**前景で動き続ける**（`Ctrl-C` で停止）。操作は**別ターミナル**で：

```bash
# コンテナに入る
docker exec -it arbeit-ros2 zsh

# ゴールを投げる(/goal_pose は自分で出す)
~/ros2_ws/src/go2_path_following/scripts/send_goal.sh 3.0 0.0   # 3m 前方へ
```

### オプション

| フラグ | 意味 |
|---|---|
| `--gate1` | GATE1計測モード。sim 付属の upstream Nav2 を止め、自作スタックだけで駆動する |
| `--no-build` | `colcon build` を飛ばす（直前のビルドを使う） |
| `-h`, `--help` | ヘルプ |

> **配線（参考）**：`/goal_pose → straight_line_planner(/plan) → plan_follower → controller_server(→/cmd_vel_raw) → cmd_vel_safety(→/robot1/cmd_vel) → ロボット`

---

## 初回セットアップ（新しいマシンで1回だけ）

`dev-up.sh` を使う前に、マシンを1回だけ構築する。**上から順に**実行する。

### ネイティブ Ubuntu

| 順 | スクリプト | 何をするか |
|---|---|---|
| ① | `ubuntu-setup.sh` | gh CLI・Docker Engine 導入、GUI許可(`xhost`)、GPU確認 |
| ② | `ubuntu-clone.sh` | `~/bridge/Go2_deploy/` に clone、submodule取得、GPU無し時の override 生成 |
| ③ | `first-run.sh` | sim/dev の**初回ビルド**＋起動＋**DDS疎通の自動チェック** |

```bash
# ① ホスト準備（実行後、ターミナルを開き直す = docker グループ反映）
curl -fsSL https://raw.githubusercontent.com/kokekokko481576/Go2_deploy/main/scripts/ubuntu-setup.sh | bash

# ② リポジトリ取得（初回は gh auth login の対話あり。ブラウザ認証が手軽）
curl -fsSL https://raw.githubusercontent.com/kokekokko481576/Go2_deploy/main/scripts/ubuntu-clone.sh | bash

# ③ 初回ビルド＋疎通確認（初回は 15〜40 分）
cd ~/bridge/Go2_deploy && ./scripts/first-run.sh
```

`first-run.sh` が以下を表示すれば成功。以降は日常起動の `dev-up.sh` を使う。

```
[OK] simコンテナから go2 が上がってきた
[OK] devコンテナから /robot1/* トピックが見える
[OK] 実データ(odometry)が dev 側に届いている
```

### Windows (WSL2)

| 順 | スクリプト | 実行場所 |
|---|---|---|
| ① | `windows-setup.ps1` | Windows(PowerShell)。WSL2導入・GPU確認、再起動 |
| ② | `wsl2-setup.sh` | WSL2内。gh CLI・Docker導入。実行後ターミナル開き直し |
| ③ | `first-run.sh` | WSL2内。初回ビルド＋疎通確認 |

```powershell
# ① Windows 側（PowerShell）
irm https://raw.githubusercontent.com/kokekokko481576/Go2_deploy/main/scripts/windows-setup.ps1 | iex
```
```bash
# ② WSL2 内
curl -fsSL https://raw.githubusercontent.com/kokekokko481576/Go2_deploy/main/scripts/wsl2-setup.sh | bash
```

> **first-run.sh と dev-up.sh の違い**：`first-run.sh` は「初回ビルド＋疎通確認」専用（初回1回）。日常の起動は `dev-up.sh`。

---

## GATE1 定量計測

GATE1（自己位置推定＋経路生成＋経路追従の合流）の到達精度を定量化する（Issue #36）。

計測系スクリプト（`send_goal.sh` / `gate1_measure.sh` / `gate1_analyze.py`）は `ros2` を呼ぶため、
**dev コンテナ内**（`ros2_ws/src/go2_path_following/scripts/`）に置いてある。

### 手順

```bash
# 1) GATE1計測モードで起動(sim の upstream Nav2 を止め、自作スタックだけで駆動)
./scripts/dev-up.sh --gate1

# 2) 別ターミナルでコンテナに入り、計測を回す
docker exec -it arbeit-ros2 zsh
cd ~/ros2_ws/src/go2_path_following/scripts
./gate1_measure.sh 50                       # 50試行
./gate1_analyze.py /tmp/gate1_results_...    # 解析
```

### `gate1_measure.sh` — 計測自動化

```bash
./gate1_measure.sh [trials]   # 省略時 50 試行（dev コンテナ内で実行）
```

ランダムゴール生成（x:1〜3m, y:-1〜1m, yaw:0〜360°）→ 投入（同ディレクトリの `send_goal.sh`）→
到達待機 → ログ記録、を繰り返す。結果は `/tmp/gate1_results_<日時>/` に試行ごとの `.log` として保存。

### `gate1_analyze.py` — 統計解析

```bash
./gate1_analyze.py /tmp/gate1_results_20260724_151200   # dev コンテナ内で実行
```

現状はログ数・サンプル表示・レポート生成。今後 xy/yaw 誤差分布・成功率・CSV/グラフ出力へ拡張予定。

---

## ゴール投入 `send_goal.sh`

`/goal_pose` に PoseStamped を publish する（YAML手打ちのミス防止）。`ros2` を呼ぶので
**dev コンテナ内**で実行する（`ros2_ws/src/go2_path_following/scripts/send_goal.sh`）。

```bash
docker exec -it arbeit-ros2 zsh
~/ros2_ws/src/go2_path_following/scripts/send_goal.sh [x] [y] [yaw(度)]

~/ros2_ws/src/go2_path_following/scripts/send_goal.sh 3.0 0.0      # 3m 前方へ、0°
~/ros2_ws/src/go2_path_following/scripts/send_goal.sh 2.0 1.5 90   # (2,1.5) へ、90°左向きに正対
```

度→ラジアン変換とクォータニオン計算を内部で行う。引数省略時は `3.0 0.0 0°`。

---

## スクリプト一覧

**ホスト側**（repo直下 `scripts/`。docker を外から操作する）

| スクリプト | 用途 |
|---|---|
| `dev-up.sh` | **日常起動**（docker〜自作スタックを一発） |
| `first-run.sh` | 初回ビルド＋起動＋DDS疎通チェック |
| `ubuntu-setup.sh` / `ubuntu-clone.sh` | ネイティブUbuntu の初回セットアップ |
| `windows-setup.ps1` / `wsl2-setup.sh` | Windows(WSL2) の初回セットアップ |

**dev コンテナ内**（`ros2_ws/src/go2_path_following/scripts/`。`ros2` を呼ぶのでコンテナ内で実行）

| スクリプト | 用途 |
|---|---|
| `send_goal.sh` | ゴール1回投入 |
| `gate1_measure.sh` / `gate1_analyze.py` | GATE1 計測・解析 |

---

## トラブルシュート

| 症状 | 対処 |
|---|---|
| `docker compose up` が `error gathering device` で失敗 | `/dev/dri` 無し環境。`compose.override.yaml.example` を `.yaml` にコピーして GPU 無効化 |
| `ubuntu-setup.sh` 後も docker が使えない | ターミナルを開き直す（docker グループ反映）。即席なら `newgrp docker` |
| Gazebo が真っ黒／激重 | GPU ソフトレンダリング。動作はするので先へ進んでよい |
| gh 認証エラー | `gh auth logout` → `ubuntu-setup.sh`/`ubuntu-clone.sh` 再実行。SSH鍵の GitHub 登録を確認 |
| ゴールを投げてもロボットが動かない | `ros2 topic echo /plan --qos-durability transient_local --once` で経路が出ているか確認（詳細は `ros2_ws/src/go2_path_following/README.md`） |

GPU無し環境の override:
```bash
cp docker/compose.override.yaml.example docker/compose.override.yaml
cp docker/sim/compose.override.yaml.example docker/sim/compose.override.yaml
```

---

## 関連ドキュメント

- `docs/手順/Ubuntuセットアップ.md` — 詳細手順・トラブルシュート
- `docs/手順/Windows-WSL2セットアップ.md` — Windows 向け詳細
- `docs/手順/デュアルブート構築.md` — Ubuntu デュアルブート構築
- `ros2_ws/src/go2_path_following/README.md` — 経路追従の使い方・詳細
- `ONBOARDING.md` — 快速スタートガイド

---

**関連 Issue:** #28/#29/#30/#36/#40/#41/#42
