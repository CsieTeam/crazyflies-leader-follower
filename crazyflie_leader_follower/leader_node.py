import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Empty
from time import sleep

class LeaderNode(Node):
    def __init__(self):
        super().__init__('leader_node')
        self.cmd_pub = self.create_publisher(PoseStamped, '/cf231/cmd_position', 10)
        self.timer = self.create_timer(2.0, self.send_next_waypoint)
        self.waypoints = [
            [0.5, 0.0, 0.5],
            [0.5, 0.5, 0.5],
            [0.0, 0.5, 0.5]
        ]
        self.step = 0
        self.takeoff_done = False
        self.land_called = False

        self.cli_takeoff = self.create_client(Empty, '/cf231/crazyflie/takeoff')
        self.cli_land = self.create_client(Empty, '/cf231/crazyflie/land')

        self.req = Empty.Request()
        self.wait_for_services()

    def wait_for_services(self):
        while not self.cli_takeoff.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for takeoff service...')
        while not self.cli_land.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for land service...')

        self.cli_takeoff.call_async(self.req)
        self.get_logger().info('Sent takeoff request')
        sleep(3.0)  # wait for takeoff to finish
        self.takeoff_done = True

    def send_next_waypoint(self):
        if not self.takeoff_done:
            return

        if self.step < len(self.waypoints):
            x, y, z = self.waypoints[self.step]
            pose = PoseStamped()
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = z
            self.cmd_pub.publish(pose)
            self.get_logger().info(f"Sending waypoint {self.step + 1}")
            self.step += 1
        elif not self.land_called:
            self.get_logger().info("Mission complete, landing...")
            self.cli_land.call_async(self.req)
            self.land_called = True

def main(args=None):
    rclpy.init(args=args)
    node = LeaderNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
