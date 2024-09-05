import rclpy

from rclpy.action import ActionClient
from rclpy.node import Node

from irobot_create_msgs.action import Dock

class DockTurtlebot(Node):
    def __init__(self, node_name: str = 'dock_tb4') -> None:
        super().__init__(node_name)
        self.get_logger().info("Dock tb4 node initialized.")
        self._undock_client = ActionClient(self, Dock, 'dock')

    def send_goal(self, instruction):
        goal_msg = Dock.Goal()

        while not self._undock_client.wait_for_server(10):
            self.get_logger().info("Waiting for the action server...")

        self._send_goal_future = self._undock_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)

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
        


def main(args= None):
    rclpy.init(args=args)
    node = DockTurtlebot()
    node.send_goal('')
    rclpy.spin(node)

if __name__ == '__main__':
    main()
