# Humble⇔Jazzy混在で出るDDS警告のしくみと対処 (Issue #7)

dev(Humble)とsim(Jazzy)を同一DDSドメインで混在させているため、launch時・CLI実行時に
大量の警告が出る。この文書は「なぜ出るのか」「実害は何か」「本リポジトリでどう対処したか」を
まとめたもの。**データそのものは正しく届いており、警告は機能面では無害**であることを
実測で確認済み(cmd_vel・odom・TF・点群すべて動作、Issue #21のGATE1動作確認も完了)。

## 症状(何が出るか)

| 症状 | 出る場所 | 見た目 |
|------|---------|-------|
| type hash警告 | Jazzy(sim)側の全ノード・CLI | `Failed to parse type hash for topic '...' from USER_DATA '(null)'` |
| rcutilsエラー上書き通知 | Humble(dev)側のノード・CLI | `>>> [rcutils\|error_handling.c:108] ... 'invalid data size' / 'string data is not null-terminated' ... <<<` のブロック |
| デシリアライズ長エラー | 両側 | `sequence size exceeds remaining buffer` |
| **ros2 daemonのグラフ破損** | 両側のCLI | `ros2 node list`等が `Failed to get node names: empty node name returned by the RMW layer` で失敗。`ros2 topic info -v` のノード名が `_NODE_NAME_UNKNOWN_` になる |
| **`ros2 topic hz`の偽陰性** | 特にdev側 | データが流れていても無出力になることがある |

量の目安: Jazzy側はノード1つあたり**10秒で約150件**のtype hash警告が出続ける(実測)。

## 原因(なぜ出るか)

ROS 2はHumble(2022)→Iron(2023)の間にRMW層へ互換性のない変更が2つ入った:

1. **型ハッシュ(REP-2011)の導入**: Iron以降のノードはエンドポイントのUSER_DATAに
   型ハッシュを載せる。Humbleのノードは載せない。Jazzy側はHumbleのエンドポイントを
   発見するたびに「type hashが読めない」と警告する(逆にHumble側のros2 daemonは
   Jazzyの型ハッシュ情報を`unknown tag 'rclpy.type_hash.TypeHash'`として処理できない)
2. **GIDサイズの変更(24→16バイト)**: ノード名解決に使う`ros_discovery_info`トピック
   (`ParticipantEntitiesInfo`型)はGIDの配列を含むが、このGIDのサイズがIronで
   24バイトから16バイト(DDS標準)に変更された。**同じ型名でワイヤ表現が異なる**ため、
   HumbleとJazzyは互いのdiscovery情報をデシリアライズできない
   (`invalid data size` / `string data is not null-terminated` の正体)

2.が「グラフ破損」の根本原因: discovery情報が読めない→相手側participantのノード名・
所属が解決できない→`_NODE_NAME_UNKNOWN_`、daemonのグラフAPIは空ノード名で例外。
**上流(ros2/rmw_cyclonedds等)に修正はなく、異ディストロ混在は公式にサポート外**。
ユーザデータ(cmd_vel等の通常トピック)は型が変わっていないので正常に流れる。

参考: [rmw_cyclonedds#487](https://github.com/ros2/rmw_cyclonedds/issues/487)、
[rclpy#1448](https://github.com/ros2/rclpy/issues/1448)、
[Iron リリースノート(GID変更)](https://docs.ros.org/en/rolling/Releases/Release-Iron-Irwini.html)

## 本リポジトリでの対処

方針: 発生源は止められない(ディストロ統一はunitree_ros2のHumble固定で不可、
ドメイン分離+ブリッジは大工事)ので、**既知の無害な警告だけをstderrから間引く**。

1. **対話シェル(`docker/common/zshrc`)**: `ros2`コマンドをラッパー関数化し、既知の
   3系統(type hash / rcutilsブロック / sequence size)をsedで間引く。dev/sim/driver
   共通。**生ログを見たいときは `ROS2_NOFILTER=1 ros2 ...`**。
   注意点が2つ、どちらも実装に織り込み済み:
   - **`ros2 launch`/`ros2 run`は子ノードのstderrを自分のstdoutに多重化して出す**
     (`[node-1] `プレフィックス付き)ため、この2サブコマンドに限りstdout側も濾す。
     他のサブコマンドのstdout(echoのデータ等)は触らない
   - launch出力は複数プロセスの行が混ざる(interleave)ので、rcutilsブロックを
     範囲(`/開始/,/終了/`)で消すのではなく、**ブロックを構成する行を1行ずつ
     パターン削除**する(範囲だと間に挟まった他ノードの正規ログを巻き込む)
2. **simのdocker logs(`docker/sim/compose.yaml`)**: launchコマンドの出力を
   `2>&1`でまとめて`grep -v`に通す(上記の多重化の事情によりstderrだけでは不足)
3. **運用上の回避(README等に記載済み)**: グラフ系CLIは`--no-daemon`を付ける、
   流量確認は`ros2 topic hz`ではなく`ros2 topic echo --once`を使う

### 意図的にやっていないこと

- **ノードごとの`--log-level rmw_cyclonedds_cpp:=error`**: Jazzy側では警告が完全に
  消えることを実験で確認済み(10秒151件→0件)だが、fork内の20以上のNode定義への
  追記が必要でupstream追従性を損なうため見送り。将来ノイズが問題になる特定ノードが
  あれば個別にこの手を使える(JazzyのlaunchにはSetROSLogLevel相当の一括手段は無い)
- **rcutilsブロックのノード側抑制**: これはロガー経由ではなくrcutilsが直接stderrへ
  印字するもので、ログレベルでは消せない(フィルタのみ)

## 注意・限界

- フィルタは「既知のパターン」だけを対象にしており、新種の警告・実エラーは素通しする
- 実測(controller.launch.py 10秒起動): ノイズ42行→0行、正規のINFOログは38行→38行で無傷
- launch/runのstdoutをsedに通す都合で、これらの出力はttyではなくパイプになる
  (色付けが変わる程度で実用上の影響は確認されていない)
- 警告そのものを調査するとき・上流にissue報告するときは必ず`ROS2_NOFILTER=1`で
  生ログを取ること
