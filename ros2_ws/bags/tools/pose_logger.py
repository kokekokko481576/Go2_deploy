# TFから map->base_link を5Hzで (時刻, x, y, yaw[deg]) に残す(再生したslamの位置を基準として使うため)
import sys, math, rclpy
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener
rclpy.init(); n=Node('pose_logger'); buf=Buffer(); TransformListener(buf,n); f=open(sys.argv[1],'w'); last=[0.0]
def tick():
    try: t=buf.lookup_transform('map','base_link',Time())
    except Exception: return
    s=t.header.stamp.sec+t.header.stamp.nanosec*1e-9
    if s<=last[0]: return
    last[0]=s; q=t.transform.rotation
    f.write('%.3f\t%.3f\t%.3f\t%.1f\n'%(s,t.transform.translation.x,t.transform.translation.y,math.degrees(math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))))); f.flush()
n.create_timer(0.05,tick)   # 壁時計0.05秒ごと。再生が等速なら約5〜20Hz
rclpy.spin(n)
