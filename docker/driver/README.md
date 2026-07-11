# driverコンテナ（unitree_ros2 + CycloneDDS 0.10.x固定）

対象: `docs/docker要件定義.md` §4.2「driver: 自作イメージ」、作業計画 Phase 0 C2「ドライバ導入」。
外部リポジトリ [unitree_ros2](https://github.com/unitreerobotics/unitree_ros2) を
`external/unitree_ros2` に git submodule として取り込み、本体は変更せず利用する。

## できること

- `unitree_go` / `unitree_api` / `unitree_hg` メッセージ定義(Go2ファームとのDDS通信I/F)
- unitree_ros2公式のサンプルノード群(`unitree_ros2_example`。go2_sport_client等)
- CycloneDDS 0.10.x(apt版 `ros-humble-rmw-cyclonedds-cpp`)固定。NIC名は`GO2_NIC`環境変数で注入(イメージには焼かない、要件R2)

## 使い方

```bash
cd docker/driver
docker compose build   # 初回のみ
docker compose up -d
docker compose exec driver bash   # 対話シェル。RMW/CycloneDDS設定は.bashrc経由で自動適用

# 実機Go2に有線LAN接続する場合(ホストで一度)
GO2_NIC=enp3s0 docker compose up -d
```

## 本体(upstream)との差分・注意点

- **`unitree_ros2_example` の `CMakeLists.txt`(upstream)に不備**: `install(TARGETS ...)` の
  destination指定が欠けており、実行ファイルがパッケージルート直下にインストールされ、
  `ros2 run`/`ros2 pkg executables` から見つけられない(バイナリ自体は正常)。
  本体は変更せず、Dockerfile側でビルド後に `lib/unitree_ros2_example/` へシンボリックリンクを
  張る後処理を追加して対処(`docker/driver/Dockerfile` 参照)
- **entrypoint.sh とCycloneDDS/RMW設定を共通化**: `/setup_dds.sh` に集約し、コンテナのメイン
  プロセス(entrypoint経由)と `docker compose exec` の対話シェル(`.bashrc` 経由)の両方から
  同じ設定が当たるようにしている。`docker exec` で非対話(`bash -c`、`-i`無し)に入ると
  Ubuntu既定の `.bashrc` の先頭ガード(`[ -z "$PS1" ] && return` 相当)により`.bashrc`の残りが
  読まれず設定が当たらない点に注意(対話シェル・`bash -ic` なら問題ない)
- **ループバック(`GO2_NIC`未指定=既定`lo`)でのDDS発見**: `lo` はmulticastフラグが
  立っておらず(本機で確認済み)、CycloneDDS既定のマルチキャストSPDP発見が機能しない。
  `setup_dds.sh` 内で `lo` の場合のみ `AllowMulticast=false` + `Peers`(ユニキャスト)+
  `ParticipantIndex=auto` に切り替えて対処(`ParticipantIndex=none`のままだとPeer側から
  ポートを特定できず発見できない)。**実機NIC使用時はこの分岐に入らず素の既定設定**
  (multicastが使える前提)になる

## 動作確認結果(2026-07-12、開発PC: Ubuntu22.04)

- イメージビルド成功(`unitree_go`/`unitree_api`/`unitree_hg`/`unitree_ros2_example` 4パッケージ)
- `ros2 doctor`: All 5 checks passed
- `ros2 pkg executables unitree_ros2_example`: 19実行ファイルすべて認識(修正後)
- `ros2 run unitree_ros2_example go2_stand_example`: 起動しセンサ読み取りループが回ることを確認(実機無しなのでゼロ値)
- ループバック(`GO2_NIC`未指定)でのpub/sub: `ros2 topic pub`→`ros2 topic echo`/`ros2 topic list`で
  受信・トピック発見を確認(daemon再起動後。上記のDDS発見設定込み)

未実施(実機・別環境が必要なため):

- 実機Go2との有線LAN接続でのDDS通信(`GO2_NIC`に実NIC名を指定しての検証)
- `unitree_ros2_example` の各サンプル(sport_client等)を実機相手に実行しての動作確認
