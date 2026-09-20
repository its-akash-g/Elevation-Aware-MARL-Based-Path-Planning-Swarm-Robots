import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

class EnergyMonitor(Node):
    def __init__(self):
        super().__init__('energy_monitor')
        self.rovers = ['rover_1', 'rover_2', 'rover_3']
        self.energy_data = {rover: 0.0 for rover in self.rovers}
        self.last_time = {rover: self.get_clock().now() for rover in self.rovers}
        
        # Base power draw (W) and movement coefficient (W per m/s)
        self.base_power = 5.0 
        self.movement_coeff = 25.0 

        self.subs = []
        for rover in self.rovers:
            sub = self.create_subscription(
                Odometry,
                f'/{rover}/diff_cont/odom',
                lambda msg, r=rover: self.odom_callback(msg, r),
                10
            )
            self.subs.append(sub)

        # Print telemetry every 2 seconds
        self.timer = self.create_timer(2.0, self.print_energy)

    def odom_callback(self, msg, rover):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time[rover]).nanoseconds / 1e9
        self.last_time[rover] = current_time

        if dt <= 0: return

        v = msg.twist.twist.linear.x
        omega = msg.twist.twist.angular.z

        # Simplified kinematic power formula
        power = self.base_power + self.movement_coeff * (abs(v) + 0.3 * abs(omega))
        
        # Energy (Joules) = Power (Watts) * time (seconds)
        self.energy_data[rover] += power * dt

    def print_energy(self):
        self.get_logger().info('\n--- Cumulative Energy Consumption ---')
        for rover in self.rovers:
            self.get_logger().info(f'{rover.upper()}: {self.energy_data[rover]:.2f} Joules')

def main():
    rclpy.init()
    node = EnergyMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
