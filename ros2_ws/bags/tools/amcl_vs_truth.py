# usage: amcl_vs_truth.py <amcl.tsv> <truth.tsv>  AMCLの位置を基準(slam)と時刻で突き合わせる
import sys,bisect,math
A=[list(map(float,l.split())) for l in open(sys.argv[1]) if l.strip()]
T=[list(map(float,l.split())) for l in open(sys.argv[2]) if l.strip()]
tt=[r[0] for r in T]; rows=[]
for a in A:
    k=min(bisect.bisect_left(tt,a[0]),len(T)-1)
    if abs(T[k][0]-a[0])>0.3: continue
    d=math.hypot(a[1]-T[k][1],a[2]-T[k][2]); dy=abs((a[3]-T[k][3]+180)%360-180)
    rows.append((a[0]-A[0][0],d,dy,math.sqrt(max(a[4],0)+max(a[5],0)),math.degrees(math.sqrt(max(a[6],0)))))
if not rows: print('比較できる行なし'); sys.exit()
ds=sorted(r[1] for r in rows); ys=sorted(r[2] for r in rows); n=len(rows)
print('比較 %d点 / 位置ずれ 中央値 %.2fm 90%%点 %.2fm 最大 %.2fm / 向きずれ 中央値 %.1f度 90%%点 %.1f度 最大 %.1f度'%(n,ds[n//2],ds[int(n*.9)],ds[-1],ys[n//2],ys[int(n*.9)],ys[-1]))
step=max(1,n//12)
for r in rows[::step]: print('  t=%4.0f 位置ずれ %.2fm 向きずれ %4.1f度 | AMCL自身の広がり σ位置 %.2fm σ向き %.1f度'%r)
r=rows[-1]; print('  最後 t=%4.0f 位置ずれ %.2fm 向きずれ %4.1f度 | σ位置 %.2fm σ向き %.1f度'%r)
