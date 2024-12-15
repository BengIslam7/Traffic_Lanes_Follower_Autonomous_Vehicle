import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import tensorflow as tf
from std_msgs.msg import Int32

class CameraPub(Node):
    def __init__(self):
        super().__init__('camera_p')

        # Créer un éditeur pour publier des messages sur le topic '/camera/image_raw'
        self.image_publisher = self.create_publisher(Image, '/camera/image_raw', 10)
        self.steering_angle_publisher = self.create_publisher(Int32, '/steering/angle', 10)

        # Initialiser le pont CvBridge pour la conversion entre OpenCV et ROS
        self.bridge = CvBridge()

        # Ouvrir la caméra (par défaut la caméra 0)
        self.cap = cv2.VideoCapture(0)

        # Load the TFLite model
	    self.interpreter = tf.lite.Interpreter(model_path='')
	    self.interpreter.allocate_tensors()

	    # Get input and output tensors
	    self.input_details = self.interpreter.get_input_details()
	    self.output_details = self.interpreter.get_output_details()

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
                
		        # Example inference
		        self.interpreter.set_tensor(self.input_details[0]['index'], frame.reshape(1, 66, 200, 3))
		        self.interpreter.invoke()
		        output_data = self.interpreter.get_tensor(self.output_details[0]['index'])
		        max_index = np.argmax(output_data)
		        print(max_index)
		        self.steering_angle_publisher.publish(max_index)
                self.get_logger().info('Steering angle publiée')
		
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

    camera_talker = CameraPub()

    try:
        rclpy.spin(camera_talker)
    except KeyboardInterrupt:
        pass

    # Clean-up
    camera_talker.destroy()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
