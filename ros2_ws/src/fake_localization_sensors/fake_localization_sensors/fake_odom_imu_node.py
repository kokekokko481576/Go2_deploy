import math

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu


class FakeOdomImuNode(Node):

    def __init__(self):
        super().__init__('fake_odom_imu_node')

        self.declare_parameter('publish_rate', 20.0)
        self.declare_parameter('linear_velocity', 0.2)
        self.declare_parameter('angular_velocity', 0.0)

        publish_rate = self.get_parameter('publish_rate').value
        self.linear_velocity = self.get_parameter('linear_velocity').value
        self.angular_velocity = self.get_parameter('angular_velocity').value

        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.imu_pub = self.create_publisher(Imu, 'imu/data', 10)

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.last_time = self.get_clock().now()

        self.timer = self.create_timer(1.0 / publish_rate, self.timer_callback)
        self.get_logger().info('FakeOdomImuNode has been started.')

    def timer_callback(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now

        self.yaw += self.angular_velocity * dt
        self.x += self.linear_velocity * math.cos(self.yaw) * dt
        self.y += self.linear_velocity * math.sin(self.yaw) * dt

        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = math.sin(self.yaw / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.yaw / 2.0)
        odom.twist.twist.linear.x = self.linear_velocity
        odom.twist.twist.angular.z = self.angular_velocity
        self.odom_pub.publish(odom)

        imu = Imu()
        imu.header.stamp = now.to_msg()
        imu.header.frame_id = 'imu_link'
        imu.orientation.z = math.sin(self.yaw / 2.0)
        imu.orientation.w = math.cos(self.yaw / 2.0)
        imu.angular_velocity.z = self.angular_velocity
        self.imu_pub.publish(imu)


def main():
    rclpy.init()
    node = FakeOdomImuNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
