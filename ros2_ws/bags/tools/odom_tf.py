import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
class N(Node):
    def __init__(s):
        super().__init__('odom_tf_replay')
        s.declare_parameter('odom_topic','/go2_localization/odometry/filtered')
        s.br=TransformBroadcaster(s)
        s.create_subscription(Odometry,s.get_parameter('odom_topic').value,s.cb,50)
    def cb(s,m):
        t=TransformStamped(); t.header=m.header; t.header.frame_id='odom'; t.child_frame_id='base_link'
        p=m.pose.pose; t.transform.translation.x=p.position.x; t.transform.translation.y=p.position.y; t.transform.translation.z=0.0
        t.transform.rotation=p.orientation; s.br.sendTransform(t)
rclpy.init(); rclpy.spin(N())
