# usage: scan_fit.py <bag> <map_yaml> <pose.tsv(時刻 x y yaw_deg ...)> [ラベル]
# 各姿勢の時刻に最も近い録画スキャンを地図上に置き、点が壁(占有セル)から10cm以内に乗る割合を出す。
# 正解位置が無くても「その姿勢でスキャンと地図が合っているか」を測れる。
import sys,sqlite3,glob,math,bisect,numpy as np
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import LaserScan
bag,myaml,ptsv=sys.argv[1:4]; label=sys.argv[4] if len(sys.argv)>4 else ptsv
import yaml,os
y=yaml.safe_load(open(myaml)); pgm=os.path.join(os.path.dirname(myaml),y['image']); res=y['resolution']; ox,oy=y['origin'][:2]
f=open(pgm,'rb'); assert f.readline().strip()==b'P5'
l=f.readline()
while l.startswith(b'#'): l=f.readline()
W,H=map(int,l.split()); f.readline(); img=np.frombuffer(f.read(),dtype=np.uint8).reshape(H,W)
occ=img<100
near=occ.copy()
for dy in range(-2,3):
    for dx in range(-2,3):
        if dx*dx+dy*dy<=4: near|=np.roll(np.roll(occ,dy,0),dx,1)
P=[list(map(float,l.split()[:4])) for l in open(ptsv) if l.strip()]
db=sorted(glob.glob(bag+'/*.db3'))[-1]; c=sqlite3.connect('file:'+db+'?mode=ro',uri=True)
tid=c.execute("select id from topics where name='/go2_localization/chin_lidar_scan'").fetchone()[0]
ts=[r[0] for r in c.execute("select timestamp from messages where topic_id=? order by timestamp",(tid,))]
# 録画時刻≒スタンプ(差は0.1秒未満)として最寄りを引く
out=[]
step=max(1,len(P)//400)
for p in P[::step]:
    k=min(bisect.bisect_left(ts,int(p[0]*1e9)),len(ts)-1)
    if abs(ts[k]-p[0]*1e9)>0.3e9: continue
    m=deserialize_message(c.execute("select data from messages where topic_id=? and timestamp=?",(tid,ts[k])).fetchone()[0],LaserScan)
    r=np.array(m.ranges); a=m.angle_min+np.arange(len(r))*m.angle_increment; ok=np.isfinite(r)&(r>0.4)&(r<8)
    if ok.sum()<20: continue
    th=math.radians(p[3]); x=p[1]+r[ok]*np.cos(a[ok]+th); yy=p[2]+r[ok]*np.sin(a[ok]+th)
    ix=((x-ox)/res).astype(int); iy=H-1-((yy-oy)/res).astype(int)
    inb=(ix>=0)&(ix<W)&(iy>=0)&(iy<H)
    hit=near[iy[inb],ix[inb]].sum()
    out.append((p[0]-P[0][0],hit/ok.sum()))
v=sorted(o[1] for o in out); n=len(v)
print('%s: %d姿勢 / 壁10cm以内の割合 中央値 %.0f%%  下位10%%点 %.0f%%  最小 %.0f%%'%(label,n,100*v[n//2],100*v[n//10],100*v[0]))
q=max(1,n//8); print('   時間順: '+' '.join('t=%.0f:%.0f%%'%(out[i][0],100*np.mean([o[1] for o in out[i:i+q]])) for i in range(0,n,q)))
