# docker/sim/tools/demo — ビード検査の流れを Gazebo で録画する（#74）

実機デモ（`tools/inspection/inspect_run.py --posture lie`）と同じ流れを Gazebo で動かし、作業報告用の動画にする。

    マーカー検知・接近 → 伏せる → アーム展開 → 手首を振って3枚撮影 → 収納 → 起立

接近は実機と同じ `marker_approach` の `approach_node` を使う。アームの姿勢は実機と同じ `inspect_poses.json` の値。
動画は画面録画ではなく、トピックから1枚に合成する。

- 全体カメラ
- 前方カメラ（AprilTag の枠・ID・距離）
- アーム先端カメラ
- 今どの段階か

最後に撮った3枚を並べる。

## 使い方

```bash
cd docker/sim && xhost +local:docker && SIM_ENABLE_NAV2=false SIM_ENABLE_RVIZ=false docker compose up -d
tools/demo/run_demo.sh            # 出力: ~/デスクトップ/Go2_sim_デモ_<日時>.mp4（約1分、1920x1080、H.264）
```

**前提: `~/marker_detection` をコンテナの `/marker_detection` にマウントしていること**
（`compose.override.yaml`。gitignore 済みのローカル設定）。
`approach_node`・`inspect_poses.json`・このディレクトリのスクリプトはそこから読む。
`tools/inspection/` と同じく、ホーム側の作業環境で動かしたものをそのまま置いている。

## 10/7 版（収納・展開の高速化・撮影位置の補正）

```bash
tools/demo/run_demo_1007.sh       # 出力: ~/デスクトップ/Go2_sim_1007_<日時>.mp4（約85秒）
```

`demo_1007.py`・`recorder_1007.py` を使う。止まったまま収納を真後ろへ回してから歩き出し（接近中はアームを固定）、
伏せて3段（`deploy_fast`）で撮影姿勢へ、アーム先端の深度カメラで `aim_correct.py` の補正を回してから撮る。
部材はわざと `MEMBER_DY=0.07`・`MEMBER_DX=0.05` ずらして置く（`spawn_scene.sh` に追加した変数）。

- sim の伏せ（STAND＋体を下げる）は8.5秒で胴体が床から約0.13m。6秒だと実機より高く、補正量が上限に掛かった
- **sim の機体が前に約29度傾いたまま戻らなくなることがある**（体を下げ続けた・途中で落ちた回のあと）。
  機体の位置を戻しても直らないので `docker compose restart` する
- 今の `compose.yaml` には `/sim_tools` のマウントが無いので、`sim_tag_bridge.py` は `/marker_detection/tools/` のものを使う

## ファイル

| ファイル | 役割 |
|---|---|
| `run_demo.sh` | 一式を起動し、録画してコンテナから取り出す |
| `spawn_scene.sh` | 部材（逆T字）と録画用の固定カメラを置く |
| `demo_flow.py` | 接近 → 伏せ → アーム → 撮影 → 収納 → 起立を進める。今の段階を `/demo_status` に出す |
| `recorder.py` | カメラ画像と `/demo_status` から動画を合成する |
| `grab_image.py` / `look_at.py` | カメラ位置合わせ用の補助 |

## sim と実機の違い

- sim の脚制御には伏せる命令がない。STAND モードで `cmd_vel.linear.z` を使い、体を下げて伏せの代わりにする
- 起立後は REST → TROT
- アームは `d1_arm_controller` に関節位置で直接指令する。一瞬で動かないよう、40度/秒で少しずつ送る
- 温度は見ていない
