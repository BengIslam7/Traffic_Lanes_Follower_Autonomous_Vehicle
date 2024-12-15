import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

class CamNode(Node):
    def __init__(self):
        super().__init__('camera_s')
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.bridge = CvBridge()

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            cv2.imshow('Camera', frame)
            cv2.waitKey(10)
        except CvBridgeError as e:
            self.get_logger().error('CvBridge Error: {0}'.format(e))

def main(args=None):
    rclpy.init(args=args)
    node = CamNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
