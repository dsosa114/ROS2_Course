import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import atexit
import os

class PathRecorder(Node):
    def __init__(self):
        super().__init__('path_recorder')
        
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.odom_subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            qos_profile
        )

        # Get the home directory
        home_directory = os.path.expanduser('~')
        # Join the home directory with the file name
        save_path = os.path.join(home_directory, 'recorded_path.txt')
        self.path_publisher = self.create_publisher(Path, '/recorded_path', 10)
        self.path = Path()
        self.path.header.frame_id = 'odom'
        
        # Open a file to write the path data
        self.output_file = open(save_path, 'w')
        self.get_logger().info('Path Recorder Node has been started.')

        # Register the function to be called on node shutdown
        atexit.register(self.shutdown_handler)

    def odom_callback(self, msg):
        # Create a PoseStamped message from the odometry pose
        pose_stamped = PoseStamped()
        pose_stamped.header = msg.header
        pose_stamped.pose = msg.pose.pose

        # Add the new pose to the path
        self.path.poses.append(pose_stamped)
        
        # Write the x and y coordinates to the text file
        x = pose_stamped.pose.position.x
        y = pose_stamped.pose.position.y
        self.output_file.write(f'{x},{y}\n')

        # Publish the path
        self.path_publisher.publish(self.path)
        
        self.get_logger().info(f'Published path with {len(self.path.poses)} waypoints.')

    def shutdown_handler(self):
        self.get_logger().info('Shutting down and closing file.')
        self.output_file.close()

def main(args=None):
    rclpy.init(args=args)
    path_recorder = PathRecorder()
    rclpy.spin(path_recorder)
    path_recorder.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()