import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraTalker(Node):
    def __init__(self):
        super().__init__('camera_talker')

        # Créer un éditeur pour publier des messages sur le topic '/camera/image_raw'
        self.image_publisher = self.create_publisher(Image, '/camera/image_raw', 10)

        # Initialiser le pont CvBridge pour la conversion entre OpenCV et ROS
        self.bridge = CvBridge()

        # Ouvrir la caméra (par défaut la caméra 0)
        self.cap = cv2.VideoCapture(0)

        # Vérifier si la caméra s'est ouverte correctement
        if not self.cap.isOpened():
            self.get_logger().error("Erreur d'ouverture de la caméra")
            exit()

        # Publier les images toutes les 0.1 secondes
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        # Lire une image de la caméra
        ret, frame = self.cap.read()
        if ret:
            # Convertir l'image OpenCV en message ROS
            try:
                ros_image = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')

                # Publier l'image
                self.image_publisher.publish(ros_image)
                self.get_logger().info('Image publiée')

            except Exception as e:
                self.get_logger().error(f"Erreur lors de la conversion de l'image : {e}")
        else:
            self.get_logger().error("Erreur de capture de l'image")

    def destroy(self):
        # Libérer la caméra et fermer toutes les fenêtres OpenCV
        self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)

    camera_talker = CameraTalker()

    try:
        rclpy.spin(camera_talker)
    except KeyboardInterrupt:
        pass

    # Clean-up
    camera_talker.destroy()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
