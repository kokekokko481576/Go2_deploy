#!/usr/bin/env bash
# scanmatcher_gap_watchdog.py(コンテナ内、root権限無し)がping不安定による入力点群の
# 途切れを検知するたびに .gap_trigger ファイルを更新する。このスクリプトはホスト側で
# それを監視し、更新を見た瞬間に短時間のtcpdumpを起動して、切断の瞬間の生パケットを
# 残す(root権限が要るtcpdumpはコンテナの中からは実行できないため)。
#
# 前提: tcpdumpをsudo無しで実行できるようcapabilityを付けておくこと(初回のみ、要sudo):
#   sudo setcap cap_net_raw,cap_net_admin=eip $(which tcpdump)
#
# 使い方: ./docker/driver/capture_on_gap.sh &
#   (real_up.sh --scanmatcher を先に起動しておくこと。Ctrl-Cで停止)
set -euo pipefail

REPO_ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
TRIGGER_FILE="$REPO_ROOT/ros2_ws/src/go2_localization/scripts/.gap_trigger"
OUT_DIR="${GAP_CAPTURE_DIR:-$HOME/go2_gap_captures}"
GO2_IP="${GO2_IP:-192.168.123.161}"
NIC="${GO2_NIC:-enp2s0}"
CAPTURE_SEC=15

mkdir -p "$OUT_DIR"

if ! getcap "$(command -v tcpdump)" 2>/dev/null | grep -q cap_net_raw; then
    echo "[capture_on_gap] tcpdumpにcapabilityが無い。先に実行すること:" >&2
    echo "  sudo setcap cap_net_raw,cap_net_admin=eip $(command -v tcpdump)" >&2
    exit 1
fi

echo "[capture_on_gap] 監視開始: $TRIGGER_FILE"
last_seen=""
while true; do
    if [ -f "$TRIGGER_FILE" ]; then
        content=$(cat "$TRIGGER_FILE" 2>/dev/null || true)
        if [ -n "$content" ] && [ "$content" != "$last_seen" ]; then
            last_seen="$content"
            gap_sec=$(echo "$content" | awk '{print $2}')
            ts=$(date +%Y%m%d_%H%M%S)
            outfile="$OUT_DIR/gap_${ts}.pcap"
            echo "[capture_on_gap] ギャップ検知(${gap_sec}秒) → ${CAPTURE_SEC}秒キャプチャ開始: $outfile"
            tcpdump -i "$NIC" host "$GO2_IP" -w "$outfile" -G "$CAPTURE_SEC" -W 1 \
                > "$OUT_DIR/gap_${ts}.tcpdump.log" 2>&1 &
            # 2026-11-12: パケットキャプチャの結果、ping切断ではなくPC側の送信停止(全DDS
            # participantが一斉に止まる)らしいと分かったため、PC側のCPU・負荷も同時記録する
            vmstat 1 "$CAPTURE_SEC" > "$OUT_DIR/gap_${ts}.vmstat.log" 2>&1 &
            top -b -d 1 -n "$CAPTURE_SEC" > "$OUT_DIR/gap_${ts}.top.log" 2>&1 &
        fi
    fi
    sleep 0.5
done
