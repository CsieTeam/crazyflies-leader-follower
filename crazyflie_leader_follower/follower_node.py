import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Empty
from time import sleep

class FollowerNode(Node):
    def __init__(self):
        super().__init__('follower_node')

        self.sub = self.create_subscription(PoseStamped, '/cf231/pose', self.follow_leader, 10)
        self.pub = self.create_publisher(PoseStamped, '/cf5/cmd_position', 10)

        self.cli_takeoff = self.create_client(Empty, '/cf5/crazyflie/takeoff')
        self.cli_land = self.create_client(Empty, '/cf5/crazyflie/land')
        self.req = Empty.Request()

        self.landed = False
        self.wait_for_services()

    def wait_for_services(self):
        while not self.cli_takeoff.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for takeoff service...')
        while not self.cli_land.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for land service...')
        self.cli_takeoff.call_async(self.req)
        self.get_logger().info('Sent takeoff request')
        sleep(3.0)

    def follow_leader(self, msg):
        if msg.pose.position.z < 0.2 and not self.landed:
            self.get_logger().info("Leader is landing, follower landing too...")
            self.cli_land.call_async(self.req)
            self.landed = True
            return

        cmd = PoseStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.pose.position.x = msg.pose.position.x - 0.3
        cmd.pose.position.y = msg.pose.position.y
        cmd.pose.position.z = msg.pose.position.z
        self.pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = FollowerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
