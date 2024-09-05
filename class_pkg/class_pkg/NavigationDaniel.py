import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from irobot_create_msgs.action import Dock, Undock


class WaypointPlanner(Node):
    def __init__(self):
        super().__init__('waypoint_planner')

        # Inicializa el cliente de acción
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.get_logger().info("Waypoint Planner Node Initialized")

        # Suscripción al tópico /clicked_point
        self.subscription = self.create_subscription(PointStamped, '/clicked_point', self.point_callback, 10)
        self.points = []
        self.get_logger().info("Waiting for two points...")
        self._undock_client = ActionClient(self,Undock,"undock")
        #self._undock_client = ActionClient(self,Undock,"undock")
    
    def send_goal(self,instruction):
        goal_msg1 = Undock.Goal()
    
        while not self._undock_client.wait_for_server(10):
            self.get_logger().info("Waiting for the action server...")

        self._send_goal_future = self._undock_client.send_goal_async(goal_msg1, feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result.is_docked))
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Received feedback: {0}'.format(feedback.sees_dock))


    def point_callback(self, msg):
        self.get_logger().info(f"Received point: ({msg.point.x}, {msg.point.y})")
        self.points.append(msg)

        if len(self.points) == 2:
            start_point = self.points[0]
            end_point = self.points[1]
            self.get_logger().info(
                f"Received two points: Start ({start_point.point.x}, {start_point.point.y}), End ({end_point.point.x}, {end_point.point.y})")

            # Limpia los puntos después de recibir los dos
            self.points = []

            # Inicia la navegación
            self.navigate_to_pose(start_point, 'Navigating to start point')
            self.navigate_to_pose(end_point, 'Navigating to end point')

    def navigate_to_pose(self, point, msg):
        self.get_logger().info(f"Preparing to {msg}")
        self._action_client.wait_for_server()

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = point.point.x
        goal_msg.pose.pose.position.y = point.point.y
        goal_msg.pose.pose.position.z = point.point.z
        goal_msg.pose.pose.orientation.w = 1.0

        # Envía el objetivo al servidor de acción
        self.get_logger().info(f"Sending goal: {msg}")
        future = self._action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        result = future.result()
        if result:
            self.get_logger().info(f"Navigation result: {result.status}")
        else:
            self.get_logger().error(f"Navigation failed")


def main(args=None):
    rclpy.init(args=args)
    node = WaypointPlanner()
    node.send_goal('')
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()