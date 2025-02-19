import cv2
import rclpy
from sensor_msgs.msg import Image
from rclpy.node import Node
from cv_bridge import CvBridge
import time

class CameraNodeSub(Node):
    def __init__(self):
        super().__init__('camera_nodesub')
        self.bridgeObject = CvBridge()
        self.topicNameFrames = 'topic_camera_image'
        self.queueSize = 10
        self.subscription = self.create_subscription(
            Image, self.topicNameFrames, self.listener_callback_function, self.queueSize
        )
        self.i = 0

    def listener_callback_function(self, imageMessage):
        self.get_logger().info(f'The image frame is received {self.i}')
        self.i += 1
        openCVImage = self.bridgeObject.imgmsg_to_cv2(imageMessage)
        cv2.imshow("Camera Video", openCVImage)
        
        # Check for key presses
        key = cv2.waitKey(1)  # Wait for 1 millisecond
        if key == 27:  # Press 'ESC' to exit
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    subscriberNode = SubscriberNodeClass()

    try:
        rclpy.spin(subscriberNode)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        subscriberNode.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
