"""ウェイポイント列から、実際に送る指令の列を組み立てる(ROS非依存)。

**送る指令はどれも、1つ前の指令から1関節しか変わらないようにする。**
多関節同時指令で j1 が可動域上限へ走る(arm_demo_node の docstring 参照)ため。
ウェイポイント間・開始姿勢への移動・中立復帰のすべてを、ここで1関節ずつに分解する。
2関節以上変わるウェイポイントは `multi_joint_rows()` で起動時に見つけて警告する
(分解の順序が意図と違う可能性があるため)。
"""

EPS = 1e-9


def changed_joints(a, b):
    """a から b で値が変わる関節の添字。"""
    return [j for j, (x, y) in enumerate(zip(a, b)) if abs(x - y) > EPS]


def multi_joint_rows(waypoints):
    """1つ前の行から2関節以上変わる行の添字(0始まり)。"""
    return [i for i in range(1, len(waypoints))
            if len(changed_joints(waypoints[i - 1], waypoints[i])) > 1]


def split_move(a, b):
    """a から b へ1関節ずつ動かす中間姿勢の列(a を含まず b を含む)。

    中立(0)へ戻す関節を添字の大きい順に先に動かし、そのあと中立から離す関節を
    添字の小さい順に動かす。既定のウェイポイント(j1で向けてからj2で俯角)を
    逆にたどる順と一致する。
    """
    cur = list(a)
    out = []
    joints = changed_joints(a, b)
    to_zero = sorted((j for j in joints if abs(b[j]) <= EPS), reverse=True)
    others = sorted(j for j in joints if abs(b[j]) > EPS)
    for j in to_zero + others:
        cur = list(cur)
        cur[j] = b[j]
        out.append(cur)
    return out


def _same(a, b):
    return not changed_joints(a, b)


def build_plan(current, waypoints, return_to_neutral, neutral):
    """current(最後に送った姿勢)から始めて送る指令の列を返す。

    - current が先頭のウェイポイントと違えば、そこへ移動する。current が
      末尾のウェイポイント(前回 `return_to_neutral:=false` で終わった姿勢)なら
      **来た道を逆にたどる**。それ以外は1関節ずつに分解する
    - ウェイポイントを順に送る
    - `return_to_neutral` なら来た道を逆にたどって先頭へ戻り、先頭が中立でなければ
      1関節ずつ中立へ
    """
    plan = []

    def push(pose):
        last = plan[-1] if plan else current
        if _same(last, pose):
            return
        for step in split_move(last, pose):
            plan.append(step)

    if not _same(current, waypoints[0]):
        if _same(current, waypoints[-1]):
            for wp in reversed(waypoints[:-1]):
                push(wp)
        else:
            push(waypoints[0])
    for wp in waypoints:
        push(wp)
    if return_to_neutral:
        for wp in reversed(waypoints[:-1]):
            push(wp)
        push(neutral)
    return plan
