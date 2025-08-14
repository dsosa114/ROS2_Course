import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import cv2

from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from rclpy.qos import QoSHistoryPolicy, QoSDurabilityPolicy

class ImageConverter(Node):
    def __init__(self):
        super().__init__('image_converter')
        self.qcar_qos_profile = QoSProfile(
				reliability   = QoSReliabilityPolicy.BEST_EFFORT,
				history 	  = QoSHistoryPolicy.KEEP_LAST,
				durability    = QoSDurabilityPolicy.VOLATILE,
				depth 		  = 10)
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            CompressedImage,
            '/qcar/rgbd_color',
            self.compressed_image_callback,
            self.qcar_qos_profile
        )
        self.publisher = self.create_publisher(Image, '/image', 10)

    def compressed_image_callback(self, msg):
        try:
            # Convert CompressedImage to OpenCV image
            cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
            # Convert OpenCV image to Image message
            image_msg = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
            self.publisher.publish(image_msg)
        except Exception as e:
            self.get_logger().error(f"Error converting image: {e}")

def main(args=None):
    rclpy.init(args=args)
    image_converter = ImageConverter()
    rclpy.spin(image_converter)
    image_converter.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == '__main__':
    main()