# map->odomの向きが変わるたびに (受信時刻, yaw[deg], x, y) をCSVに追記する
import rclpy,sys,math,time
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
rclpy.init(); n=Node('yaw_logger'); f=open(sys.argv[1],'w'); prev=[None]
def cb(m):
    for t in m.transforms:
        if t.header.frame_id=='map' and t.child_frame_id=='odom':
            q=t.transform.rotation; y=math.degrees(math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)))
            k=(round(y,2),round(t.transform.translation.x,3))
            if k!=prev[0]:
                prev[0]=k; s=t.header.stamp.sec+t.header.stamp.nanosec*1e-9
                f.write('%.2f,%.2f,%.3f,%.3f\n'%(s,y,t.transform.translation.x,t.transform.translation.y)); f.flush()
n.create_subscription(TFMessage,'/tf',cb,100)
rclpy.spin(n)
