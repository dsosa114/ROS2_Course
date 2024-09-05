import rclpy
from rclpy.node import Node
from functools import partial
import numpy as np
import cv2
import cv_bridge

from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import TeleportAbsolute
from std_srvs.srv import Empty


class TurtlesimController(Node):
    def __init__(self):
        super().__init__("turtlesim_controller")
        self.get_logger().info("turtlesim_controller node has started " + str(cv2.__version__))
        self.abs_client = self.create_client(TeleportAbsolute, "turtle1/teleport_absolute")
        self.clear_path_client = self.create_client(Empty, "clear")
        self.initialize_turtle()

        self.publisher_ = self.create_publisher(Twist, "turtle1/cmd_vel", 10)
        self.pose_subscriber_ = self.create_subscription(Pose, "turtle1/pose", self.pose_subscription_callback, 10)
        # self.timer_ = self.create_timer(0.05, self.timer_callback)
        
        self.x = 1.
        self.y = 1.
        self.theta = 0.

        self.x_goal = 9.5
        self.y_goal = 5
        self.theta_goal = np.deg2rad(90)

        self.kp_linear = 9.35e-1
        self.kd_linear = 1.85e-1
        self.ki_linear = 0.

        self.kp_angular = 5.35e-1
        self.kd_angular = 2.75e-1
        self.ki_angular = 0.

        self.prev_error_position = 0
        self.prev_error_angle = 0

        self.time_prev = self.get_clock().now().nanoseconds

    def timer_callback(self):
        dist = self.get_distance(np.array([self.x, self.y]), np.array([self.x_goal, self.y_goal]))
        angle = self.theta_goal - self.theta
        cmd = Twist()
        if dist > 0.25:
            v, w = self.PID(dist, angle)
            cmd.linear.x = v
            cmd.linear.y = 0.
            cmd.angular.z = w
            
        else:
            cmd.linear.x = 0.
            cmd.linear.y = 0.
            cmd.angular.z = 0.

        self.publisher_.publish(cmd)
        self.get_logger().info(f"x: {self.x}, y: {self.y}, theta: {self.theta}, dist: {dist}, dt: {angle}")


    def pose_subscription_callback(self, msg):
        time_now= self.get_clock().now().nanoseconds
        time = (time_now - self.time_prev)*1e-9
        self.time_prev = time_now

        self.x = msg.x
        self.y = msg.y
        self.theta = msg.theta
        # l_vel = msg.linear_velocity
        # a_vel = msg.angular_velocity

        dist = self.get_distance(np.array([self.x, self.y]), np.array([self.x_goal, self.y_goal]))
        angle = self.theta_goal - self.theta
        cmd = Twist()
        if dist > 0.17:
            v, w = self.PID(dist, angle)
            cmd.linear.x = v
            cmd.linear.y = 0.
            cmd.angular.z = w
            
        else:
            cmd.linear.x = 0.
            cmd.linear.y = 0.
            cmd.angular.z = 0.

        self.publisher_.publish(cmd)
        self.get_logger().info(f"x: {self.x}, y: {self.y}, theta: {self.theta}, dist: {dist}, dt: {angle}")        

        

    def teleport_absolute_callback(self, future):
        try:
            self.clear_path_client.call_async(Empty.Request())
            self.get_logger().info(f"Turtle ready in start position.")
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))

    def get_distance(self, pt_0, pt_f):
        return np.linalg.norm(pt_0 - pt_f)
    
    def get_angle(x1, y1, x2, y2):
	    # return np.arctan2(y2 - y1, x2 - x1)
        return np.arctan2(y2 - y1, x2 - x1)

    def PID(self, error_position, error_angle):
        linear_velocity_control = self.kp_linear*error_position + self.kd_linear*(error_position - self.prev_error_position)
        angular_velocity_control = self.kp_angular*error_angle + self.kd_angular*(error_angle - self.prev_error_angle)
        
        self.prev_error_angle = error_angle
        self.prev_error_position = error_position

        return linear_velocity_control, angular_velocity_control

    def initialize_turtle(self):
        while not self.abs_client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Server TeleportAbsolute")
        
        request = TeleportAbsolute.Request()
        request.x = 1.0
        request.y = 1.0
        request.theta = 0.

        future = self.abs_client.call_async(request)
        future.add_done_callback(self.teleport_absolute_callback)
        return True

def main(args=None):
    rclpy.init(args=args)
    node = TurtlesimController()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()