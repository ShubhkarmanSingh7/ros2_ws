import cv2
import rclpy
from sensor_msgs.msg import Image
from rclpy.node import Node
from cv_bridge import CvBridge
import time

class PublisherNodeClass(Node):
    def __init__(self, camera_index=0):
        super().__init__('publisher_node')
        self.camera_index = camera_index  # Use a single camera index
        self.camera = self.open_camera(self.camera_index)
        self.bridgeObject = CvBridge()
        self.topicNameFrames = 'topic_camera_image'
        self.queueSize = 10
        self.publisher = self.create_publisher(Image, self.topicNameFrames, self.queueSize)
        self.periodCommunication = 0.02
        self.timer = self.create_timer(self.periodCommunication, self.timer_callback_function)
        self.i = 0

    def open_camera(self, index):
        cap = cv2.VideoCapture(index)
        if not cap.isOpened():
            self.get_logger().error(f"Failed to open camera {index}.")
            return None
        self.get_logger().info(f"Camera {index} opened successfully.")
        return cap

    def timer_callback_function(self):
        if self.camera is None:
            self.get_logger().error("Camera is not initialized. Exiting callback.")
            return

        success, frame = self.camera.read()
        if success:
            frame = cv2.resize(frame, (820, 640), interpolation=cv2.INTER_CUBIC)
            ROS2ImageMessage = self.bridgeObject.cv2_to_imgmsg(frame)
            self.publisher.publish(ROS2ImageMessage)
            self.get_logger().info('Publishing image number %d' % self.i)
            self.i += 1
        else:
            self.get_logger().error("Failed to read frame from camera.")
        time.sleep(0.5)


def main(args=None):
    rclpy.init(args=args)
    publisherNode = PublisherNodeClass()
    rclpy.spin(publisherNode)
    if publisherNode.camera:
        publisherNode.camera.release()
    publisherNode.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

