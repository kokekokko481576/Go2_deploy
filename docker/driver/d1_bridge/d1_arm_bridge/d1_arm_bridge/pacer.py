"""送信間隔の下限を守りながら、指令を**捨てずに順番どおり**送るための待ち行列。**ROS に依存しない。**

下限に満たない指令を捨てると、上流(d1_arm_demo)が送ったウェイポイントが黙って抜け、
アームが途中の姿勢に留まったまま上流が完了を報告する。最新の1件だけを残す形にもしない。
上流は1関節ずつの指令を順に送っており(多関節同時指令で j1 が可動域上限へ走るため)、
途中を飛ばすと複数関節が同時に変わる指令になってしまう。
"""

from collections import deque


class CommandPacer:

    def __init__(self, min_interval, max_pending):
        self.min_interval = min_interval
        self.max_pending = max_pending
        self.pending = deque()
        self.last_sent = None

    def push(self, item):
        """待ち行列に積む。満杯なら積まずに False を返す。"""
        if len(self.pending) >= self.max_pending:
            return False
        self.pending.append(item)
        return True

    def wait_time(self, now):
        """次を送れるようになるまでの秒数(0 なら今送れる)。"""
        if self.last_sent is None:
            return 0.0
        return max(0.0, self.last_sent + self.min_interval - now)

    def pop_ready(self, now):
        """今送ってよい指令を1件取り出す。無ければ None。"""
        if not self.pending or self.wait_time(now) > 0.0:
            return None
        self.last_sent = now
        return self.pending.popleft()

    def bypass(self, now):
        """下限を無視して今すぐ送る場合(ゼロ姿勢への復帰)。待っていた指令は捨てる。

        復帰のあとで古い指令が送られると、戻したアームがまた動き出すため。
        捨てた件数を返す。
        """
        dropped = len(self.pending)
        self.pending.clear()
        self.last_sent = now
        return dropped
