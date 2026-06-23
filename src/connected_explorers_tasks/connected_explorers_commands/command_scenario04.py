import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class CmdVelTestSequence(Node):
    def __init__(self):
        super().__init__('cmd_vel_test_sequence')
        
        self.publisher_ = self.create_publisher(Twist, 'robot1/ideal_cmd_vel', 10)

        # ---------------------------------------------------------
        # Define your test sequence here:
        # Format: (linear_x, linear_y, angular_z, duration (seconds))
        # ---------------------------------------------------------
        self.sequence = [
            (0.0, 1.0, 0.0, 7.0),   # Step 1: Move forward (X) for 3s
            (0.0, 0.0, 0.0, 1.0),   # Step 2: Stop for 1s
            (-1.0, 0.0, 0.0, 6.0),   # Step 1: Move forward (X) for 3s
            (0.0, 0.0, 0.0, 1.0),   # Step 2: Stop for 1s
            (0.0, 0.0, -1.0, 4.0),   # Step 1: Move forward (X) for 3s
            (0.0, 0.0, 0.0, 1.0),   # Step 2: Stop for 1s
            (0.0, -1.0, 0.0, 30.0),   # Step 1: Move forward (X) for 3s
            (0.0, 0.0, 0.0, 1.0), 
        ]

        self.current_step = 0
        self.step_start_time = self.get_clock().now()

        # Timer running at 10Hz
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info('Starting holonomic cmd_vel test sequence...')

    def timer_callback(self):
        if self.current_step >= len(self.sequence):
            self.get_logger().info('Test sequence completed.')
            
            # Publish zeros to ensure the robot stops safely
            self.publisher_.publish(Twist())
            
            self.timer.cancel()
            raise SystemExit 

        # Unpack the 4 variables including linear_y
        linear_x, linear_y, angular_z, duration = self.sequence[self.current_step]

        now = self.get_clock().now()
        elapsed_time = (now - self.step_start_time).nanoseconds / 1e9 

        if elapsed_time >= duration:
            self.get_logger().info(f'Step {self.current_step + 1} complete.')
            self.current_step += 1
            self.step_start_time = self.get_clock().now()
            return

        # Build and publish the Twist message
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.linear.y = float(linear_y)
        msg.linear.z = float(angular_z)
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelTestSequence()
    
    try:
        rclpy.spin(node)
    except SystemExit:
        rclpy.logging.get_logger("Quitting").info('Sequence finished successfully.')
    except KeyboardInterrupt:
        rclpy.logging.get_logger("Quitting").info('Sequence interrupted by user.')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()