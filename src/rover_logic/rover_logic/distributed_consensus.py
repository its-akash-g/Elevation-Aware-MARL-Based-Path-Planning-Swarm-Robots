import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
import math

class DistributedConsensusNode(Node):
    def __init__(self):
        super().__init__('distributed_consensus')
        
        self.namespaces = ['rover_1', 'rover_2', 'rover_3']
        
        # Swarm Global Target (Change this to test different shortest paths)
        self.global_target_x = 5.0
        self.global_target_y = 5.0
        
        # MARL State Variables
        self.energy_levels = {'rover_1': 100.0, 'rover_2': 100.0, 'rover_3': 100.0}
        self.current_positions = {'rover_1': (0.0, 0.0), 'rover_2': (0.0, 0.0), 'rover_3': (0.0, 0.0)}
        
        # DCS Weights: w1 (Distance/Shortest Path), w2 (Energy Preservation)
        self.w1 = 1.0 
        self.w2 = 5.0
        
        self.nav_clients = {}
        self.odom_subs = {}
        
        for ns in self.namespaces:
            # Action clients for Nav2 Obstacle Avoidance
            self.nav_clients[ns] = ActionClient(self, NavigateToPose, f'/{ns}/navigate_to_pose')
            # Subscribers for real-time position tracking
            self.odom_subs[ns] = self.create_subscription(
                Odometry, f'/{ns}/odom', 
                lambda msg, rover_id=ns: self.odom_callback(msg, rover_id), 10)
            
        # Control loop evaluates MARL consensus every 4 seconds
        self.timer = self.create_timer(4.0, self.marl_consensus_loop)
        self.get_logger().info("MARL Distributed Consensus Node Initialized.")

    def odom_callback(self, msg, rover_id):
        # Update internal state with actual Gazebo coordinates
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.current_positions[rover_id] = (x, y)

    def calculate_distance(self, x1, y1, x2, y2):
        return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

    def marl_consensus_loop(self):
        costs = {}
        
        # 1. MARL Algorithm: Calculate Cost Function C_i for each rover
        for ns in self.namespaces:
            # Shortest Path Component (Distance to goal)
            curr_x, curr_y = self.current_positions[ns]
            dist_to_goal = self.calculate_distance(curr_x, curr_y, self.global_target_x, self.global_target_y)
            
            # Energy Component (Inverse energy - lower battery means higher cost)
            energy_factor = 100.0 / max(1.0, self.energy_levels[ns])
            
            # Final DCS Cost Equation
            costs[ns] = (self.w1 * dist_to_goal) + (self.w2 * energy_factor)
            
        # 2. Leader Election: argmin C_i
        leader = min(costs, key=costs.get)
        optimal_cost = costs[leader]
        
        # Simulate active energy drain for the moving leader
        self.energy_levels[leader] = max(0.0, self.energy_levels[leader] - 2.5)
        
        # 3. Terminal Output for Verification
        self.get_logger().info(
            f"\n[MARL CONSENSUS ALGORITHM]"
            f"\n -> Target Goal : ({self.global_target_x}, {self.global_target_y})"
            f"\n -> Elected Ldr : {leader.upper()}"
            f"\n -> Optimal C_i : {optimal_cost:.2f}"
            f"\n -> Dist to Goal: {dist_to_goal:.2f}m"
            f"\n -> Energy Dist : R1={self.energy_levels['rover_1']:.1f}% | "
            f"R2={self.energy_levels['rover_2']:.1f}% | "
            f"R3={self.energy_levels['rover_3']:.1f}%"
            f"\n--------------------------------------------------"
        )
        
        # 4. Dispatch the Leader via Nav2 for Shortest Path Execution
        self.send_nav2_goal(leader, self.global_target_x, self.global_target_y)

    def send_nav2_goal(self, rover_id, x, y):
        client = self.nav_clients[rover_id]
        if not client.wait_for_server(timeout_sec=0.5):
            return
            
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = float(x)
        goal.pose.position.y = float(y)
        goal.pose.orientation.w = 1.0
        
        client.send_goal_async(goal)

def main(args=None):
    rclpy.init(args=args)
    node = DistributedConsensusNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
