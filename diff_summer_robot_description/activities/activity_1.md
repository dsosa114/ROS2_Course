# Activity: ROS2 Inter-Node Image Processing

This activity will guide you through creating two ROS2 nodes that work together. One node will capture video from a webcam, and the second node will receive and process that video in real-time.

---

## Learning Objectives

Upon successful completion of this activity, you will be able to:

* Create a ROS2 package with Python nodes.
* Develop a **publisher node** that uses OpenCV to capture images from a camera.
* Use `cv_bridge` to convert OpenCV images (`cv::Mat`) into ROS2 messages (`sensor_msgs/msg/Image`).
* Develop a **subscriber node** that receives image messages.
* Use `cv_bridge` to convert ROS2 image messages back into OpenCV images for processing.
* Build and run two nodes simultaneously to establish a complete image-processing pipeline.

---

## Instructions

### Part 1: Setting up the Package

1.  **Navigate to your Workspace**: Open a terminal and go to the `src` directory of your ROS2 workspace.
    ```bash
    cd ~/ros2_ws/src
    ```

2.  **Create the ROS2 Package**: Create a new Python package named `ros2_inter_node_practice` with the necessary dependencies.
    ```bash
    ros2 pkg create --build-type ament_python ros2_inter_node_practice --dependencies rclpy sensor_msgs cv_bridge image_transport
    ```

### Part 2: Creating the Image Publisher Node

1.  **Navigate to the Package Directory**:
    ```bash
    cd ~/ros2_ws/src/ros2_inter_node_practice/ros2_inter_node_practice
    ```

2.  **Create the Python File**: Create a new file named `camera_publisher.py`.
    ```bash
    touch camera_publisher.py
    ```

3.  **Add the Code**: Open `camera_publisher.py` and paste the following code into it. This node will open your webcam, capture frames, and publish them to the `/camera/image_raw` topic.

    ```python
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Image
    from cv_bridge import CvBridge
    import cv2

    class CameraPublisher(Node):
        def __init__(self):
            super().__init__('camera_publisher')
            self.publisher_ = self.create_publisher(Image, '/camera/image_raw', 10)
            timer_period = 0.05  # seconds (for ~20 FPS)
            self.timer = self.create_timer(timer_period, self.timer_callback)
            self.cap = cv2.VideoCapture(0) # Use 0 for the default webcam
            self.br = CvBridge()
            self.get_logger().info('Camera Publisher Node has been started.')

        def timer_callback(self):
            ret, frame = self.cap.read()
            if ret:
                self.publisher_.publish(self.br.cv2_to_imgmsg(frame, 'bgr8'))
                # self.get_logger().info('Publishing video frame') # Uncomment for debugging
            else:
                self.get_logger().warn('Could not read frame from camera.')

    def main(args=None):
        rclpy.init(args=args)
        camera_publisher = CameraPublisher()
        rclpy.spin(camera_publisher)
        camera_publisher.destroy_node()
        rclpy.shutdown()

    if __name__ == '__main__':
        main()
    ```

### Part 3: Creating the Image Processing Node

1.  **Create the Python File**: In the same directory, create a file named `image_processor.py`.
    ```bash
    touch image_processor.py
    ```

2.  **Add the Code**: Open `image_processor.py` and paste the following code. This node subscribes to the `/camera/image_raw` topic, converts the images to grayscale, and displays them.

    ```python
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Image
    from cv_bridge import CvBridge
    import cv2

    class ImageProcessor(Node):
        def __init__(self):
            super().__init__('image_processor')
            self.subscription = self.create_subscription(
                Image,
                '/camera/image_raw',
                self.listener_callback,
                10)
            self.br = CvBridge()
            self.get_logger().info('Image Processor Node has been started.')

        def listener_callback(self, data):
            # self.get_logger().info('Receiving video frame') # Uncomment for debugging
            current_frame = self.br.imgmsg_to_cv2(data, "bgr8")
            
            # --- Image Processing Logic ---
            gray_frame = cv2.cvtColor(current_frame, cv2.COLOR_BGR2GRAY)
            
            # Display the processed frame
            cv2.imshow("Grayscale Feed", gray_frame)
            cv2.waitKey(1)

    def main(args=None):
        rclpy.init(args=args)
        image_processor = ImageProcessor()
        rclpy.spin(image_processor)
        image_processor.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()

    if __name__ == '__main__':
        main()
    ```

### Part 4: Building and Running the Nodes

1.  **Configure `setup.py`**: Open the `setup.py` file located in `~/ros2_ws/src/ros2_inter_node_practice/` and add the `entry_points` to register your nodes as executables.
    ```python
    # Inside the setup() function, add:
    entry_points={
        'console_scripts': [
            'camera_pub = ros2_inter_node_practice.camera_publisher:main',
            'img_proc = ros2_inter_node_practice.image_processor:main',
        ],
    },
    ```

2.  **Build the Package**: Navigate to the root of your workspace and use `colcon` to build your new package.
    ```bash
    cd ~/ros2_ws
    colcon build --packages-select ros2_inter_node_practice
    ```

3.  **Source the Workspace**: In every new terminal you use, you must source your workspace's setup file.
    ```bash
    source install/setup.bash
    ```

4.  **Run the Nodes**:
    * In your **first terminal**, source the workspace and run the camera publisher:
        ```bash
        source ~/ros2_ws/install/setup.bash
        ros2 run ros2_inter_node_practice camera_pub
        ```
    * Open a **second terminal**, source the workspace, and run the image processor:
        ```bash
        source ~/ros2_ws/install/setup.bash
        ros2 run ros2_inter_node_practice img_proc
        ```

You should now see a window titled "Grayscale Feed" displaying the live, processed video from your webcam.
