import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
import tf_transformations
import numpy as np 

def create_pose_stamped(navigator: BasicNavigator, position_x, position_y, orientation_z):
    q_x, q_y, q_z, q_w = tf_transformations.quaternion_from_euler(0.0, 0.0, np.deg2rad(orientation_z))
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = position_x
    pose.pose.position.y = position_y
    pose.pose.position.z = 0.0
    pose.pose.orientation.x = q_x
    pose.pose.orientation.y = q_y
    pose.pose.orientation.z = q_z
    pose.pose.orientation.w = q_w
    
    return pose

def main():
    # --- Init
    rclpy.init()
    nav = BasicNavigator()

    # --- Set initial pose
    q_x, q_y, q_z, q_w = tf_transformations.quaternion_from_euler(0.0, 0.0, 0.0)
    initial_pose = create_pose_stamped(nav, 0.0, 0.0, 0.0)
    nav.setInitialPose(initial_pose)

    # --- Wait for Nav2
    nav.waitUntilNav2Active()

    # --- Send Nav2 goal
    goal_pose1 = create_pose_stamped(nav, 4.0, 2.5, 135.)
    goal_pose2 = create_pose_stamped(nav, 1.0, 5.0, 180.)
    goal_pose3 = create_pose_stamped(nav, -3.0, 2.0, -45.)

    # --- Go to one pose
    nav.goToPose(goal_pose1)

    while not nav.isTaskComplete():
        feedback = nav.getFeedback()
        nav.get_logger().info(f'x: {feedback.current_pose.pose.position.x}, y: {feedback.current_pose.pose.position.y}')

    # --- Follow waypoints
    waypoints = [goal_pose1, goal_pose2, goal_pose3, initial_pose]
    nav.followWaypoints(waypoints)

    while not nav.isTaskComplete():
        feedback = nav.getFeedback()
        nav.get_logger().info(f'waypoint: {feedback.current_waypoint}')
    # --- ShutDown
    rclpy.shutdown()

if __name__=='__main__':
    main()
