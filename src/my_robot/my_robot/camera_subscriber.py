import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraListener(Node):
    def __init__(self):
        super().__init__('camera_listener')
        self.get_logger().info('Camera listener has been started.')

        # Créer un abonné au topic de la caméra
        self.image_subscriber = self.create_subscription(
            Image,
            '/camera/image_raw',  # Assurez-vous que c'est le bon topic de la caméra
            self.listener_callback,
            10  # Taille de la file d'attente
        )

        self.bridge = CvBridge()

    def listener_callback(self, msg):
        self.get_logger().info('Received an image.')

        try:
            # Convertir l'image ROS en format OpenCV (par exemple en BGR8)
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            
            # Affichage de l'image avec OpenCV
            cv2.imshow("Camera Feed", cv_image)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f"Could not convert image: {e}")

def main(args=None):
    rclpy.init(args=args)

    camera_listener = CameraListener()

    rclpy.spin(camera_listener)

    camera_listener.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()