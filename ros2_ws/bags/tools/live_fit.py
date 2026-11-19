# usage: live_fit.py [map_yaml] [tape_x tape_y tape_yaw_rad]
#        live_fit.py --selftest <bag> [map_yaml]   (録画済みbagの最初のスキャンで計算だけ確かめる)
# ライブで、AMCLの推定位置(TFの map->スキャン座標系)にいまのスキャンを置き、
# 点が地図の壁から10cm以内に乗る割合を1秒ごとに表示する(scan_fit.pyのライブ版)。
# あわせて開始位置テープからのずれ(距離・向き)も出す。テープに戻したときの値が位置誤差の目安。
# 購読するだけで、何も配信しない。記録は /tmp/live_amcl/fit.tsv(時刻 x y yaw_deg 割合 有効本数)。
import sys, os, math, time, yaml, numpy as np

SCAN = '/go2_localization/chin_lidar_scan'


class Grid:
    def __init__(self, myaml):
        y = yaml.safe_load(open(myaml))
        self.res = y['resolution']; self.ox, self.oy = y['origin'][:2]
        f = open(os.path.join(os.path.dirname(os.path.abspath(myaml)), y['image']), 'rb')
        assert f.readline().strip() == b'P5'
        l = f.readline()
        while l.startswith(b'#'): l = f.readline()
        self.W, self.H = map(int, l.split()); f.readline()
        occ = np.frombuffer(f.read(), dtype=np.uint8).reshape(self.H, self.W) < 100
        near = occ.copy()
        for dy in range(-2, 3):
            for dx in range(-2, 3):
                if dx * dx + dy * dy <= 4: near |= np.roll(np.roll(occ, dy, 0), dx, 1)
        self.near = near

    def fit(self, ranges, amin, ainc, x, y, th):
        """(壁10cm以内の割合, 有効本数)。有効20本未満は (None, 本数)"""
        r = np.asarray(ranges, dtype=float); a = amin + np.arange(len(r)) * ainc
        ok = np.isfinite(r) & (r > 0.4) & (r < 8)
        n = int(ok.sum())
        if n < 20: return None, n
        px = x + r[ok] * np.cos(a[ok] + th); py = y + r[ok] * np.sin(a[ok] + th)
        ix = ((px - self.ox) / self.res).astype(int); iy = self.H - 1 - ((py - self.oy) / self.res).astype(int)
        inb = (ix >= 0) & (ix < self.W) & (iy >= 0) & (iy < self.H)
        return float(self.near[iy[inb], ix[inb]].sum()) / n, n


def selftest(bag, myaml, tape):
    import sqlite3, glob
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import LaserScan
    g = Grid(myaml)
    c = sqlite3.connect('file:' + sorted(glob.glob(bag + '/*.db3'))[-1] + '?mode=ro', uri=True)
    tid = c.execute("select id from topics where name=?", (SCAN,)).fetchone()[0]
    for (d,) in c.execute("select data from messages where topic_id=? order by timestamp limit 5", (tid,)):
        m = deserialize_message(d, LaserScan)
        print('テープ位置: %s / 0.5m横にずらす: %s / 30度回す: %s  (frame=%s)' % (
            g.fit(m.ranges, m.angle_min, m.angle_increment, *tape),
            g.fit(m.ranges, m.angle_min, m.angle_increment, tape[0] + 0.5, tape[1], tape[2]),
            g.fit(m.ranges, m.angle_min, m.angle_increment, tape[0], tape[1], tape[2] + 0.52), m.header.frame_id))


def live(myaml, tape):
    import rclpy, tf2_ros
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan
    g = Grid(myaml)
    rclpy.init(); n = Node('live_fit'); last = [None, 0.0]
    def cb(m): last[0] = m; last[1] = time.time()
    n.create_subscription(LaserScan, SCAN, cb, qos_profile_sensor_data)
    buf = tf2_ros.Buffer(); tf2_ros.TransformListener(buf, n)
    os.makedirs('/tmp/live_amcl', exist_ok=True); log = open('/tmp/live_amcl/fit.tsv', 'a')
    print('地図 %s / テープ (%.3f, %.3f, %.1f度)。Ctrl-Cで終了' % (myaml, tape[0], tape[1], math.degrees(tape[2])))
    nxt = time.time() + 1
    try:
        while rclpy.ok():
            rclpy.spin_once(n, timeout_sec=0.1)
            if time.time() < nxt: continue
            nxt = time.time() + 1
            m = last[0]
            if m is None or time.time() - last[1] > 2:
                print('スキャンが届いていません(%s)' % ('未受信' if m is None else '%.0f秒前が最後' % (time.time() - last[1]))); continue
            try:
                tr = buf.lookup_transform('map', m.header.frame_id, rclpy.time.Time()).transform
            except Exception as e:
                print('TF map->%s が引けません: %s' % (m.header.frame_id, str(e)[:80])); continue
            q = tr.rotation; th = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            x, y = tr.translation.x, tr.translation.y
            f, k = g.fit(m.ranges, m.angle_min, m.angle_increment, x, y, th)
            dth = math.degrees((th - tape[2] + math.pi) % (2 * math.pi) - math.pi)
            print('位置 (%+.3f, %+.3f) 向き %+6.1f度 | 重なり %s 有効%3d本 | テープから %.3fm %+.1f度' % (
                x, y, math.degrees(th), '  --' if f is None else '%3.0f%%' % (100 * f), k, math.hypot(x - tape[0], y - tape[1]), dth))
            log.write('%.2f\t%.3f\t%.3f\t%.1f\t%s\t%d\n' % (time.time(), x, y, math.degrees(th), '' if f is None else '%.3f' % f, k)); log.flush()
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    a = sys.argv[1:]; DEF = '/home/ros/ros2_ws/map_1118e_ang.yaml'; TAPE = (-0.068, -0.107, -0.325)
    if a and a[0] == '--selftest':
        selftest(a[1], a[2] if len(a) > 2 else DEF, TAPE)
    else:
        live(a[0] if a else DEF, tuple(map(float, a[1:4])) if len(a) >= 4 else TAPE)
