import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Header
import cv2
from cv_bridge import CvBridge

class CameraNode(Node):
    """A ROS 2 Node that captures images from a camera and publishes them."""

    def __init__(self):
        """Initialize the camera node and start publishing images."""
        super().__init__('camera_node')
        self.publisher_ = self.create_publisher(Image, 'camera/image', 10)
        self.timer = self.create_timer(0.033, self.publish_camera_image)  # ~30 FPS
        self.bridge = CvBridge()

        # Initialize OpenCV video capture
        self.cap = cv2.VideoCapture(0)  # Change index if needed (0 is usually the default camera)

        if not self.cap.isOpened():
            self.get_logger().error('Failed to open camera.')
            raise RuntimeError('Camera not accessible')

        self.get_logger().info("Camera Node has been started with OpenCV.")

    def publish_camera_image(self):
        """Capture an image from the camera and publish it as a ROS Image message."""
        ret, frame = self.cap.read()

        if not ret:
            self.get_logger().error('Failed to capture image from camera')
            return

        try:
            # Convert image to ROS2 Image message
            msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            msg.header = Header()
            msg.header.stamp = self.get_clock().now().to_msg()

            self.publisher_.publish(msg)
            self.get_logger().debug('Published camera image.')
        except Exception as e:
            self.get_logger().error(f'Error converting image: {e}')

    def destroy_node(self):
        """Stop the video capture and clean up resources when the node is destroyed."""
        self.cap.release()  # Release the camera
        super().destroy_node()
        self.get_logger().info("Camera Node has been shut down and camera released.")

def main(args=None):
    """Main entry point for the Camera Node."""
    rclpy.init(args=args)
    camera_node = CameraNode()

    try:
        rclpy.spin(camera_node)
    except KeyboardInterrupt:
        pass
    finally:
        camera_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()


    
