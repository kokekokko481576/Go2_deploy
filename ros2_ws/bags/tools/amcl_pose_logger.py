# amcl_pose を (スタンプ, x, y, yaw[deg], 分散xx, 分散yy, 分散yaw) でTSVに残す
import sys, math, rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.qos import QoSProfile, DurabilityPolicy
def yaw(q): return math.degrees(math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)))
rclpy.init(); n=Node('amcl_pose_logger'); f=open(sys.argv[1],'w')
def cb(m):
    p=m.pose.pose; c=m.pose.covariance; t=m.header.stamp.sec+m.header.stamp.nanosec*1e-9
    f.write('%.3f\t%.3f\t%.3f\t%.1f\t%.4f\t%.4f\t%.4f\n'%(t,p.position.x,p.position.y,yaw(p.orientation),c[0],c[7],c[35])); f.flush()
n.create_subscription(PoseWithCovarianceStamped,'/go2_localization/amcl_pose',cb,QoSProfile(depth=10,durability=DurabilityPolicy.TRANSIENT_LOCAL))
rclpy.spin(n)
