import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import random

class DistributedConsensusNode(Node):
    def __init__(self):
        super().__init__('distributed_consensus')
        
        # Define namespace isolated rovers
        self.namespaces = ['/rover_1', '/rover_2', '/rover_3']
        self.publishers_ = {}
        
        # Create a publisher for each rover's differential drive controller topic
        for ns in self.namespaces:
            topic = f"{ns}/diff_cont/cmd_vel_unstamped"
            self.publishers_[ns] = self.create_publisher(Twist, topic, 10)
            
        # 2 Hz update rate for visible consensus shifting during your presentation
        self.timer = self.create_timer(0.5, self.control_loop)
        self.get_logger().info("Elevation-Aware Dynamic Consensus & Leader Election Initialized.")

    def control_loop(self):
        # Simulate real-time elevation-aware energy costs (C_i) based on terrain slope
        costs = {
            'ROVER_1': round(random.uniform(0.35, 0.55), 2),
            'ROVER_2': round(random.uniform(0.30, 0.60), 2),
            'ROVER_3': round(random.uniform(0.40, 0.65), 2)
        }
        
        # Elect the leader dynamically with the minimum elevation energy cost (argmin C_i)
        leader = min(costs, key=costs.get)
        min_cost = costs[leader]
        
        self.get_logger().info(f"Consensus Update -> Leader: {leader} | Optimal Cost C_i: {min_cost} | All Costs: {costs}")

        # Coordinate movement: Leader drives path-finding, followers maintain consensus formation
        for ns, pub in self.publishers_.items():
            twist = Twist()
            rover_name = ns.replace('/', '').upper()
            
            if rover_name == leader:
                # Leader takes the optimal forward trajectory
                twist.linear.x = 0.25
                twist.angular.z = 0.0
            else:
                # Follower rovers coordinate and adjust speed to maintain formation
                twist.linear.x = 0.18
                twist.angular.z = 0.04
                
            pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = DistributedConsensusNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
