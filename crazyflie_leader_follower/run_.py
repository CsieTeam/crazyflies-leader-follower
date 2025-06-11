import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import Empty
from geometry_msgs.msg import PoseStamped
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

        self.cli_takeoff = self.create_client(Empty, '/cf231/takeoff')
        self.cli_land = self.create_client(Empty, '/cf231/land')
        self.req = Empty.Request()

        self.cli_takeoff.call_async(self.req)
        self.get_logger().info('Sent takeoff request to cf231')
        sleep(3.0)
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


class FollowerNode(Node):
    def __init__(self):
        super().__init__('follower_node')
        self.sub = self.create_subscription(PoseStamped, '/cf231/cmd_position', self.follow_leader, 10)
        self.pub = self.create_publisher(PoseStamped, '/cf5/cmd_position', 10)

        self.cli_takeoff = self.create_client(Empty, '/cf5/takeoff')
        self.cli_land = self.create_client(Empty, '/cf5/land')
        self.req = Empty.Request()

        self.landed = False

        self.cli_takeoff.call_async(self.req)
        self.get_logger().info('Sent takeoff request to cf5')
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


# Entry point
def main(args=None):
    rclpy.init(args=args)
    leader_node = LeaderNode()
    follower_node = FollowerNode()

    executor = MultiThreadedExecutor()
    executor.add_node(leader_node)
    executor.add_node(follower_node)

    try:
        executor.spin()
    finally:
        leader_node.destroy_node()
        follower_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()