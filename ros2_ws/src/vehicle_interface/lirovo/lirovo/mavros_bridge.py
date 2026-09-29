#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped


class MavrosOdomBridge(Node):
    def __init__(self):
        super().__init__('mavros_odom_bridge')
        self.latest_odom_msg = None
        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT

        self.odom_pub = self.create_publisher(Odometry, '/odom', qos)

        self.tf_broadcaster = TransformBroadcaster(self)

        self.subscription = self.create_subscription(
            Odometry,
            '/odometry/filtered',
            self.odom_callback,
            qos
        )

        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def odom_callback(self, odom_msg: Odometry):
        self.latest_odom_msg = odom_msg

    def timer_callback(self):
        if self.latest_odom_msg is None:
            return

        current_time = self.get_clock().now().to_msg()

        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose = self.latest_odom_msg.pose
        odom.twist = self.latest_odom_msg.twist
        self.odom_pub.publish(odom)

        tf = TransformStamped()
        tf.header.stamp = current_time
        tf.header.frame_id = 'odom'
        tf.child_frame_id = 'base_link'
        tf.transform.translation.x = odom.pose.pose.position.x
        tf.transform.translation.y = odom.pose.pose.position.y
        tf.transform.translation.z = odom.pose.pose.position.z
        tf.transform.rotation = odom.pose.pose.orientation

        self.tf_broadcaster.sendTransform(tf)


def main(args=None):
    rclpy.init(args=args)
    odom_bridge = MavrosOdomBridge()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(odom_bridge)

    try:
        executor.spin()
    except KeyboardInterrupt:
        print("Shutting down MAVROS Bridge...")
    finally:
        odom_bridge.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
