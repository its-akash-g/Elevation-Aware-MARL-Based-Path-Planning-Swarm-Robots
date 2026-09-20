import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import math

class DistributedConsensus(Node):
    def __init__(self):
        super().__init__('distributed_consensus')
        self.rovers = ['rover_1', 'rover_2', 'rover_3']
        
        # Target objective for the fleet
        self.target_x = 15.0
        self.target_y = 15.0
        
        self.positions = {r: {'x': 0.0, 'y': 0.0, 'z': 0.0, 'theta': 0.0} for r in self.rovers}
        self.costs = {r: float('inf') for r in self.rovers}
        self.leader = 'rover_1'

        self.pubs = {}
        self.subs = []

        for rover in self.rovers:
            self.pubs[rover] = self.create_publisher(Twist, f'/{rover}/diff_cont/cmd_vel_unstamped', 10)
            self.subs.append(
                self.create_subscription(Odometry, f'/{rover}/diff_cont/odom', 
                lambda msg, r=rover: self.odom_callback(msg, r), 10)
            )

        # Control loop at 10Hz
        self.timer = self.create_timer(0.1, self.control_loop)

    def euler_from_quaternion(self, x, y, z, w):
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        return math.atan2(t3, t4)

    def odom_callback(self, msg, rover):
        pos = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.positions[rover]['x'] = pos.x
        self.positions[rover]['y'] = pos.y
        self.positions[rover]['z'] = pos.z
        self.positions[rover]['theta'] = self.euler_from_quaternion(q.x, q.y, q.z, q.w)

        # Cost Function: Distance to target + Heavy penalty for elevation (z)
        dist_to_target = math.hypot(self.target_x - pos.x, self.target_y - pos.y)
        self.costs[rover] = (1.0 * dist_to_target) + (25.0 * max(0.0, pos.z))

    def control_loop(self):
        # 1. Consensus: Elect the leader with the lowest traversal cost
        valid_costs = {r: c for r, c in self.costs.items() if c != float('inf')}
        if not valid_costs:
            self.get_logger().info('Waiting for odometry data from Gazebo...', throttle_duration_sec=2.0)
            return
            
        self.leader = min(valid_costs, key=valid_costs.get)
        self.get_logger().info(f'Current Leader: {self.leader.upper()} | Cost: {self.costs[self.leader]:.2f}')

        # 2. Command Execution
        for rover in self.rovers:
            msg = Twist()
            x, y, theta = self.positions[rover]['x'], self.positions[rover]['y'], self.positions[rover]['theta']
            
            if rover == self.leader:
                # Leader moves to the main target
                goal_x, goal_y = self.target_x, self.target_y
            else:
                # Followers move toward the leader
                goal_x, goal_y = self.positions[self.leader]['x'], self.positions[self.leader]['y']

            distance = math.hypot(goal_x - x, goal_y - y)
            angle_to_goal = math.atan2(goal_y - y, goal_x - x)
            angle_error = angle_to_goal - theta

            # Normalize angle error to [-pi, pi]
            angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))

            if distance > 0.5:
                msg.linear.x = min(0.5, 0.5 * distance)
                msg.angular.z = max(-1.0, min(1.0, 1.5 * angle_error))
            
            self.pubs[rover].publish(msg)

def main():
    rclpy.init()
    node = DistributedConsensus()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
