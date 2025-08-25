import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np

class CompressedImageViewer(Node):
    def __init__(self):
        super().__init__('compressed_image_viewer')

        # 1. Declare the parameter with a default value
        self.declare_parameter('subscribe_topic', '/qcar/csi_front')
        
        # 2. Get the parameter value
        topic_name = self.get_parameter('subscribe_topic').get_parameter_value().string_value
        self.get_logger().info(f'Subscribing to topic: {topic_name}')

        self.subscription = self.create_subscription(
            CompressedImage,
            topic_name,  # Change this to your compressed image topic
            self.image_callback,
            10
        )
        self.subscription  # prevent unused variable warning

    def image_callback(self, msg):
        try:
            # The msg.data is a byte array, so we convert it to a NumPy array
            np_arr = np.frombuffer(msg.data, np.uint8)
            # Decode the image data using OpenCV
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

            if cv_image is not None:
                # Display the image in a window named 'Compressed Image'
                cv2.imshow('Compressed Image', cv_image)
                cv2.waitKey(1)  # The waitKey(1) is necessary for the window to update
            else:
                self.get_logger().error('Could not decode the compressed image.')

        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')

def main(args=None):
    rclpy.init(args=args)
    compressed_image_viewer = CompressedImageViewer()
    rclpy.spin(compressed_image_viewer)

    # Destroy the node and shutdown ROS
    compressed_image_viewer.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()