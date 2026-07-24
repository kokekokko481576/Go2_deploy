# docs/ 索引

自己位置推定・経路生成・経路追従の3計画を横断するドキュメント一式。迷ったら**現在の進捗は
[`作業計画.md`](作業計画.md)の「履歴」節が正**、実際に手を動かす手順は
[`開発ガイド.md`](開発ガイド.md)を見る、の2つを覚えておけば足りる。

## 現状・進め方

| ファイル | 内容 |
|---------|------|
| [`作業計画.md`](作業計画.md) | 3計画を横断した全体の依存関係・進捗管理(**進捗の正**) |
| [`開発ガイド.md`](開発ガイド.md) | 実践寄りの手引き。今どこまで来たか・次に何をするか・開発の基本サイクル・ハマりがちな罠集 |
| [`docker要件定義.md`](docker要件定義.md) | Docker環境(dev/sim/driver)の要件定義・進捗ログ |
| [`go2_rl_ros2_overview.md`](go2_rl_ros2_overview.md) | 跨ぎスキル(Isaac Lab強化学習)など、本3計画とは別の担当ラインの超入門ガイド |

## 計画(何を作るか — 目標設計)

各サブシステムの目的・マイルストーン・詳細設計。

- [`計画/自己位置推定.md`](計画/自己位置推定.md)
- [`計画/経路生成.md`](計画/経路生成.md)
- [`計画/経路追従.md`](計画/経路追従.md)

## 解説(なぜそう作ったか — 実装の背景)

初心者向けの噛み砕いた解説と、外部依存の扱い方。

- [`解説/自己位置推定のしくみ.md`](解説/自己位置推定のしくみ.md) — EKF+AMCLが何をしているかのやさしい解説
- [`解説/外部サブモジュールの使い方.md`](解説/外部サブモジュールの使い方.md) — git submodule(fork/upstream運用)の解説
- [`解説/Humble-Jazzy混在DDS警告.md`](解説/Humble-Jazzy混在DDS警告.md) — launch時の大量警告の原因(型ハッシュ・GID変更)と対処(issue #7)

## 手順(どう確認するか — 動作確認手順書)

- [`手順/Docker動作確認.md`](手順/Docker動作確認.md) — devコンテナの起動・動作確認の手順書
- [`手順/Ubuntuセットアップ.md`](手順/Ubuntuセットアップ.md) — ネイティブUbuntu(26.04含む)の環境構築「これをやるだけ」版(issue #40)
- [`手順/デュアルブート構築.md`](手順/デュアルブート構築.md) — Windows機にUbuntu 26.04を追加するデュアルブート手順(issue #40)
- [`手順/Windows-WSL2セットアップ.md`](手順/Windows-WSL2セットアップ.md) — Windows機での環境構築「これをやるだけ」版(issue #28)

## 各パッケージのREADME(実装の詳細)

パッケージ単位の詳しいI/F・設計判断・動作確認結果は、`docs/`ではなく各パッケージ内に置いてある
(このリポジトリの慣習)。

- [`ros2_ws/src/go2_localization/README.md`](../ros2_ws/src/go2_localization/README.md)
- [`ros2_ws/src/go2_path_following/README.md`](../ros2_ws/src/go2_path_following/README.md)
- [`ros2_ws/src/straight_line_planner/README.md`](../ros2_ws/src/straight_line_planner/README.md)
- [`ros2_ws/src/cmd_vel_safety/README.md`](../ros2_ws/src/cmd_vel_safety/README.md)
- [`ros2_ws/src/fake_localization_sensors/README.md`](../ros2_ws/src/fake_localization_sensors/README.md)
- [`docker/README.md`](../docker/README.md)・[`docker/sim/README.md`](../docker/sim/README.md)・[`docker/driver/README.md`](../docker/driver/README.md)・[`docker/common/README.md`](../docker/common/README.md)
