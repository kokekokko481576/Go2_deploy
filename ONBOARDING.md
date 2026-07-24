# Go2_deploy オンボーディング

Unitree Go2（顎3D LiDAR・背面アーム搭載）で船内部材へ移動し溶接・検査を行うロボットシステム。
**自己位置推定・経路生成・経路追従**の3サブシステムを ROS2 Humble + Nav2 で統合する。
開発方針は Isaac Lab（学習）→ Gazebo（リハーサル）→ 実機Go2 の **シミュレーション先行**。

このガイドは「新しく入った人が **sim を起動して経路追従を動かす** ところまで最短で辿り着く」ためのもの。

---

## 0. これだけ知っておく

| 知りたいこと | 見る場所 |
|-------------|---------|
| いま何がどこまで出来ているか（進捗の正） | `docs/作業計画.md` の「履歴」節 |
| 実際に手を動かす手引き・ハマりどころ | `docs/開発ガイド.md` |
| ドキュメント全体の索引 | `docs/README.md` |
| 各機能の詳細I/F・設計判断 | 各パッケージ内の `README.md`（`ros2_ws/src/*/README.md`） |

- 環境は **3つの Docker**：`docker/`（dev: 開発・Nav2）、`docker/sim/`（Gazebo）、`docker/driver/`（実機）。
- **Gazebo は新Gazebo(Fortress)**。コマンドは `ign gazebo`（`gz` ではない）。
- リポジトリは **PUBLIC**。clone は `--recurse-submodules` 付きで。`chapter1` は取得されなくて正常。

---

## 1. セットアップ（OS別）

### Windows（WSL2） ← いま整備・検証中

**WSL2ネイティブ運用**（WSL2内のUbuntuにDocker Engineを直接入れる。Docker Desktopは使わない）。
詳細版は `docs/手順/Windows-WSL2セットアップ.md`。

> **前提**: ストレージ空き **30〜40GB 推奨**（最低25GB。sim/devイメージ約13.6GB + WSL基盤 +
> 初回ビルド中の一時キャッシュ+10〜15GB）、**RAM 16GB以上 推奨**（`.wslconfig`でWSL2に12GB割当）。

最短は次の3本を上から実行するだけ：

1. GPUドライバをベンダー公式から最新に更新（AMDなら Adrenalin。ここだけ手動）
2. **PowerShell（管理者）**:
   ```powershell
   irm https://raw.githubusercontent.com/kokekokko481576/Go2_deploy/main/scripts/windows-setup.ps1 | iex
   ```
   （WSL新規導入なら再起動 → Ubuntu初期ユーザ作成まで）
3. **Ubuntu(WSL2)ターミナル**:
   ```bash
   curl -fsSL https://raw.githubusercontent.com/kokekokko481576/Go2_deploy/main/scripts/wsl2-setup.sh | bash
   ```
   [OK]/[NG]の自己診断つき。NGは指示に従い解消 → 再実行（何度でも安全）
4. **ターミナルを開き直して**:
   ```bash
   cd ~/Go2_deploy && ./scripts/first-run.sh
   ```
   ビルド（初回15〜40分）→ 起動 → DDS疎通チェックまで自動。

> ⚠️ clone は **必ず WSL2側のLinuxFS（`~/` 以下）**。`/mnt/c/...` は CRLF混入・bind mount低速化で不可。
> 詰まったら詰まり方を issue に記録（GUI=#29 / DDS疎通=#30 / 実機Go2=#31）。**Windowsは検証進行中**なので報告が価値になる。

### Ubuntu（動作確認済み・基準環境）

> Ubuntu をまだ入れていない／デュアルブートで用意するなら → `docs/手順/デュアルブート構築.md`。
> **26.04 でもOK**（コンテナ中身は Ubuntu22.04 固定なので ROS2 はホストのUbuntu版に非依存）。

最短は次の3本を上から実行するだけ（gh CLI・Docker 導入から clone・初回ビルドまで自動）：

1. **ホスト側の準備**（gh CLI・Docker Engine 導入、`xhost` を rc に追記）:
   ```bash
   curl -fsSL https://raw.githubusercontent.com/kokekokko481576/Go2_deploy/main/scripts/ubuntu-setup.sh | bash
   ```
   終わったら**ターミナルを開き直す**（docker グループ反映）。
2. **リポジトリ取得**（gh で `~/bridge/Go2_deploy` へ。初回は gh の SSH 認証の対話が入る）:
   ```bash
   curl -fsSL https://raw.githubusercontent.com/kokekokko481576/Go2_deploy/main/scripts/ubuntu-clone.sh | bash
   ```
3. **初回ビルド+起動+疎通チェック**:
   ```bash
   cd ~/bridge/Go2_deploy && ./scripts/first-run.sh
   ```

いずれも[OK]/[NG]自己診断つきで**何度でも安全**。詳細・gh認証の答え方・トラブルシュートは
`docs/手順/Ubuntuセットアップ.md`。GUI（RViz2/Gazebo）は 26.04 だと Wayland 既定なので、
出ないときは Xorg でログインし直す。

### macOS

GUIは XQuartz 経由、実機DDSはホストネットワーク制約あり。詳細は `docker/README.md`。

---

## 2. 動いたか確かめる

```bash
# devコンテナから sim のトピックが見えれば DDS疎通OK
docker exec arbeit-ros2 bash -c 'source /opt/ros/humble/setup.bash && ros2 topic list' | grep /robot1/
```
GazeboのGUIウィンドウ（カフェ+ロボット）が出て、上のトピックが見えれば環境は完成。
次の一歩は経路追従の動作確認 → `ros2_ws/src/go2_path_following/README.md` の「使い方(フェーズB)」。
ゴール投入は `scripts/send_goal.sh <x> <y> <yaw度>`。

---

## 3. 開発の約束ごと

- **コミット**: 和文セマンティック形式 `<type>: <絵文字> <#issue> <和文要約>`
  （例: `feat: ✨ #16 planner_serverを追加`）。`Co-Authored-By` は付けない。
- **issue**: 実装・検証の説明は issue コメントに書く。close できそうなら確認の上で閉じる。
- **作業ブランチ**: 各自ブランチ（例 `feat/localization` / `feat/planning` / `feat/following`）で、
  `ros2_ws/src/<担当パッケージ>/` と**別パッケージに分けて**作業（お互い干渉しない）。
- **main へは PR 経由**。安定版を @kokekokko481576 が承認してマージ。
- **協働の全ルール（体制・アクセス・引き継ぎ方針）は固定 issue #41 を参照**。
- `ros2_ws/{build,install,log}` は gitignore 済み。`chapter1/` は触らない。

---

## 4. 困ったら

- Gazeboが真っ黒/激重 → コンテナ内GPU無効（ソフトレンダリング）。動作はするので先へ進んでよい（`docs/手順/Windows-WSL2セットアップ.md` トラブルシュート）。
- launch時に大量のDDS警告 → Humble⇔Jazzy混在の既知現象（`docs/解説/Humble-Jazzy混在DDS警告.md`、#7）。
- `ros2 topic list` で何も見えない → 両コンテナが同じホスト/WSL2内か、`ROS_DOMAIN_ID=0` かを確認。
- ROS2 CLI診断の罠: `ros2 topic hz` は偽陰性が出やすい。`ros2 node list` は `--no-daemon` 必須。最も信頼できるのは `ros2 topic echo`。
