# 録画の/tfから slam_toolbox の map->base_link を (時刻, x, y, yaw[deg]) で取り出す(AMCLとの比較用の基準)
import sqlite3,sys,glob,math
from rclpy.serialization import deserialize_message
from tf2_msgs.msg import TFMessage
def yaw(q): return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
db=sorted(glob.glob(sys.argv[1]+'/*.db3'))[-1]; out=open(sys.argv[2],'w')
c=sqlite3.connect('file:'+db+'?mode=ro',uri=True)
tid=c.execute("select id from topics where name='/tf'").fetchone()[0]
mo=None;last=0
for ts,data in c.execute("select timestamp,data from messages where topic_id=? order by timestamp",(tid,)):
    for t in deserialize_message(data,TFMessage).transforms:
        tr=t.transform
        if t.header.frame_id=='map' and t.child_frame_id=='odom': mo=(tr.translation.x,tr.translation.y,yaw(tr.rotation))
        elif t.header.frame_id=='odom' and t.child_frame_id=='base_link' and mo:
            s=t.header.stamp.sec+t.header.stamp.nanosec*1e-9
            if s-last<0.1: continue
            last=s; ox,oy,oyaw=tr.translation.x,tr.translation.y,yaw(tr.rotation)
            x=mo[0]+math.cos(mo[2])*ox-math.sin(mo[2])*oy; y=mo[1]+math.sin(mo[2])*ox+math.cos(mo[2])*oy
            out.write('%.3f\t%.3f\t%.3f\t%.1f\n'%(s,x,y,math.degrees((mo[2]+oyaw+math.pi)%(2*math.pi)-math.pi)))
