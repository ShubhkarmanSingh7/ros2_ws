import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, Image
from gpiozero import Motor, RotaryEncoder

class MotorControl(Node):
    def __init__(self):
        super().__init__('motor_control')

        # Set up subscribers for cmd_vel (velocity commands), LIDAR, and Camera data
        self.subscription = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.lidar_subscription = self.create_subscription(LaserScan, 'lidar_node/scan', self.lidar_callback, 10)
        self.camera_subscription = self.create_subscription(Image, 'camera_node/image_raw', self.camera_callback, 10)

        # Define motor connections (replace GPIO pins with your actual wiring)
        self.motor_a = Motor(forward=17, backward=18)  # Motor A control pins
        self.motor_b = Motor(forward=22, backward=23)  # Motor B control pins

        # Define encoder connections (replace GPIO pins with your actual wiring)
        self.encoder_a = RotaryEncoder(24, 25)  # Encoder A pins
        self.encoder_b = RotaryEncoder(26, 27)  # Encoder B pins (if needed)

        # Initialize variables for LIDAR and camera data
        self.lidar_ranges = []
        self.image_data = None

    def lidar_callback(self, msg):
        """Process incoming LIDAR scan data."""
        self.lidar_ranges = msg.ranges
        self.get_logger().info(f"LIDAR ranges received: {len(self.lidar_ranges)} points")
        self.avoid_obstacle()  # Call obstacle avoidance logic after receiving LIDAR data

    def camera_callback(self, msg):
        """Process incoming camera image data."""
        self.image_data = msg
        self.get_logger().info("Received camera frame.")
        # You can add camera-based processing here, like object detection

    def cmd_vel_callback(self, msg):
        """Process incoming velocity commands (manual control)."""
        linear_velocity = msg.linear.x
        angular_velocity = msg.angular.z

        self.get_logger().info(f"Received cmd_vel: linear={linear_velocity}, angular={angular_velocity}")

        # Move the robot based on cmd_vel topic commands
        if linear_velocity > 0:
            self.move_forward(linear_velocity)
        elif linear_velocity < 0:
            self.move_backward(-linear_velocity)
        else:
            self.stop_motors()

        if angular_velocity != 0:
            self.turn(angular_velocity)

    def avoid_obstacle(self):
        """Basic obstacle avoidance based on LIDAR data."""
        if self.lidar_ranges:
            # Split LIDAR data into left and right halves for obstacle detection
            left_distances = self.lidar_ranges[:len(self.lidar_ranges) // 2]
            right_distances = self.lidar_ranges[len(self.lidar_ranges) // 2:]

            avg_left = sum(left_distances) / len(left_distances)
            avg_right = sum(right_distances) / len(right_distances)

            self.get_logger().info(f"LIDAR Data: Min={min(self.lidar_ranges)}, Avg Left={avg_left}, Avg Right={avg_right}")

            if min(self.lidar_ranges) < 0.5:  # Stop if an obstacle is too close
                self.get_logger().info("Obstacle too close, stopping.")
                self.stop_motors()
            elif avg_left < avg_right:  # Turn right if there's more space on the right
                self.get_logger().info("Turning right to avoid obstacle.")
                self.turn(-0.3)
            elif avg_right < avg_left:  # Turn left if there's more space on the left
                self.get_logger().info("Turning left to avoid obstacle.")
                self.turn(0.3)
            else:  # Move forward if there's no close obstacle
                self.get_logger().info("Path clear, moving forward.")
                self.move_forward(0.5)

    def move_forward(self, speed):
        """Move forward with specified speed."""
        self.motor_a.forward(speed)
        self.motor_b.forward(speed)
        self.get_logger().info(f'Moving forward at speed {speed}')

    def move_backward(self, speed):
        """Move backward with specified speed."""
        self.motor_a.backward(speed)
        self.motor_b.backward(speed)
        self.get_logger().info(f'Moving backward at speed {speed}')

    def stop_motors(self):
        """Stop both motors."""
        self.motor_a.stop()
        self.motor_b.stop()
        self.get_logger().info('Motors stopped.')

    def turn(self, angular_velocity):
        """Turn the car based on the angular velocity."""
        if angular_velocity > 0:  # Turn left
            self.motor_a.forward(0.5)
            self.motor_b.backward(0.5)
            self.get_logger().info(f'Turning left with angular velocity {angular_velocity}')
        elif angular_velocity < 0:  # Turn right
            self.motor_a.backward(0.5)
            self.motor_b.forward(0.5)
            self.get_logger().info(f'Turning right with angular velocity {angular_velocity}')


def main(args=None):
    rclpy.init(args=args)

    # Create the motor control node
    motor_control = MotorControl()

    # Keep the node running and responsive to callbacks
    rclpy.spin(motor_control)

    # Cleanup and shutdown
    motor_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


     

   
