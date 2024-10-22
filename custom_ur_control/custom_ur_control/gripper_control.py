import rclpy
import copy 

import numpy as np

from rclpy.node import Node
from rclpy.action import ActionClient, ActionServer
from rclpy.action import CancelResponse, GoalResponse

from sensor_msgs.msg import JointState

import asyncio

from control_msgs.action import GripperCommand
from action_msgs.msg import GoalStatus
from custom_ur_interfaces.action import GripperPosition


class GripperPositionControl(Node):
    def __init__(self, node_name: str = 'gripper_controller', *, context: rclpy.Context | None = None, cli_args: rclpy.List[str] | None = None, namespace: str | None = None, use_global_arguments: bool = True, enable_rosout: bool = True, start_parameter_services: bool = True, parameter_overrides: rclpy.List[rclpy.Parameter] | None = None, allow_undeclared_parameters: bool = False, automatically_declare_parameters_from_overrides: bool = False) -> None:
        super().__init__(node_name, context=context, cli_args=cli_args, namespace=namespace, use_global_arguments=use_global_arguments, enable_rosout=enable_rosout, start_parameter_services=start_parameter_services, parameter_overrides=parameter_overrides, allow_undeclared_parameters=allow_undeclared_parameters, automatically_declare_parameters_from_overrides=automatically_declare_parameters_from_overrides)

        # Create action clients for multiple servers
        self._action_client_LKJ = ActionClient(self, GripperCommand, '/robotiq_controller_LKJ/gripper_cmd')
        self._action_client_RKJ = ActionClient(self, GripperCommand, '/robotiq_controller_RKJ/gripper_cmd')
        self._action_client_LIKJ = ActionClient(self, GripperCommand, '/robotiq_controller_LIKJ/gripper_cmd')
        self._action_client_RIKJ = ActionClient(self, GripperCommand, '/robotiq_controller_RIKJ/gripper_cmd')
        self._action_client_LFTJ = ActionClient(self, GripperCommand, '/robotiq_controller_LFTJ/gripper_cmd')
        self._action_client_RFTJ = ActionClient(self, GripperCommand, '/robotiq_controller_RFTJ/gripper_cmd')

        # Create an action server in this same node
        self._action_server = ActionServer(
            self,
            GripperPosition,
            '/robotiq_controller_full',
            self.execute_callback
        )

        self.joint_state_sub = self.create_subscription(
                JointState, "joint_states", self.joint_state_callback, 10
            )
        
        self.joint_state_msg_received = False
        self.current_position = None

    ### Action Server - Receive the goal and relay it to Action Clients ###
    async def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal on /robotiq_controller_full...')
        # Relay the goal received from the server to both action clients
        goal_received = goal_handle.request
        # self.get_logger().info(f'Received: {goal_received}')
        gripper_action = goal_received.action
        self.get_logger().info(f'Received gripper action: {gripper_action}')

        goal_msg = GripperCommand.Goal()
        # goal_msg.command = goal_received.command
        # goal_msg.command.max_effort = goal_received.max_effort

        goal_msg_inv = copy.deepcopy(goal_msg)
        # goal_msg_inv.command.position = -1. * goal_msg.command.position
        feedback_msg = GripperPosition.Feedback()

        if np.isclose(goal_received.command.position, self.current_position):
            self.get_logger().info('The gripper has reached the desirted position.')
            goal_handle.succeed()

            result = GripperPosition.Result()
            result.position = self.current_position
            result.effort = 0.0
            result.stalled = False
            result.reached_goal = True
            return result

        elif self.current_position < goal_received.command.position:
            while self.current_position < goal_received.command.position:
                step_position = self.current_position + 0.1
                if step_position >= 0.8:
                    step_position = 0.8
                goal_msg.command.position = step_position
                goal_msg.command.max_effort = goal_received.command.max_effort
                goal_msg_inv.command.position = -1. * goal_msg.command.position

                futures = [
                    # Send goals to action clients and wait for their results
                    self._send_goal_to_client(self._action_client_LKJ, goal_msg, 'server_LKJ'),
                    self._send_goal_to_client(self._action_client_RKJ, goal_msg_inv, 'server_RKJ'),
                    self._send_goal_to_client(self._action_client_LIKJ, goal_msg, 'server_LIKJ'),
                    self._send_goal_to_client(self._action_client_RIKJ, goal_msg_inv, 'server_RIKJ'),
                    self._send_goal_to_client(self._action_client_LFTJ, goal_msg_inv, 'server_LFTJ'),
                    self._send_goal_to_client(self._action_client_RFTJ, goal_msg, 'server_RFTJ')
                ]
                # Wait for both clients' futures to complete
                results = await self._wait_for_futures(futures)

                result_client_LKJ, result_client_RKJ, result_client_LIKJ, result_client_RIKJ, result_client_LFTJ, result_client_RFTJ = results
                # Process the results and determine the outcome
                if result_client_LKJ and result_client_RKJ and result_client_LIKJ and result_client_RIKJ and result_client_LFTJ and result_client_RFTJ:
                    feedback_msg.position = step_position
                    feedback_msg.stalled = False

                    self.joint_state_msg_received = False
                else:
                    feedback_msg.position = self.current_position
                    feedback_msg.stalled = True
                    break

                goal_handle.publish_feedback(feedback_msg)
                # self._sleep_async(0.001)
        else:
            while self.current_position > goal_received.command.position:
                step_position = self.current_position - 0.1
                if step_position <= 0.0:
                    step_position = 0.0
                goal_msg.command.position = step_position
                goal_msg.command.max_effort = goal_received.command.max_effort
                goal_msg_inv.command.position = -1. * goal_msg.command.position

                futures = [
                    # Send goals to action clients and wait for their results
                    self._send_goal_to_client(self._action_client_LKJ, goal_msg, 'server_LKJ'),
                    self._send_goal_to_client(self._action_client_RKJ, goal_msg_inv, 'server_RKJ'),
                    self._send_goal_to_client(self._action_client_LIKJ, goal_msg, 'server_LIKJ'),
                    self._send_goal_to_client(self._action_client_RIKJ, goal_msg_inv, 'server_RIKJ'),
                    self._send_goal_to_client(self._action_client_LFTJ, goal_msg_inv, 'server_LFTJ'),
                    self._send_goal_to_client(self._action_client_RFTJ, goal_msg, 'server_RFTJ')
                ]
                # Wait for both clients' futures to complete
                results = await self._wait_for_futures(futures)

                result_client_LKJ, result_client_RKJ, result_client_LIKJ, result_client_RIKJ, result_client_LFTJ, result_client_RFTJ = results
                # Process the results and determine the outcome
                if result_client_LKJ and result_client_RKJ and result_client_LIKJ and result_client_RIKJ and result_client_LFTJ and result_client_RFTJ:
                    feedback_msg.position = step_position
                    feedback_msg.stalled = False

                    self.joint_state_msg_received = False
                else:
                    feedback_msg.position = self.current_position
                    feedback_msg.stalled = True
                    break

                goal_handle.publish_feedback(feedback_msg)
                # self._sleep_async(0.001)
        
        goal_handle.succeed()
        result = GripperPosition.Result()
        result.position = self.current_position
        result.effort = 0.0
        result.stalled = False
        result.reached_goal = True
        return result
        
        # futures = [
        #     # Send goals to action clients and wait for their results
        #     self._send_goal_to_client(self._action_client_LKJ, goal_msg, 'server_LKJ'),
        #     self._send_goal_to_client(self._action_client_RKJ, goal_msg_inv, 'server_RKJ'),
        #     self._send_goal_to_client(self._action_client_LIKJ, goal_msg, 'server_LIKJ'),
        #     self._send_goal_to_client(self._action_client_RIKJ, goal_msg_inv, 'server_RIKJ'),
        #     self._send_goal_to_client(self._action_client_LFTJ, goal_msg_inv, 'server_LFTJ'),
        #     self._send_goal_to_client(self._action_client_RFTJ, goal_msg, 'server_RFTJ')
        # ]
        # # Wait for both clients' futures to complete
        # results = await self._wait_for_futures(futures)
        # # result_client_LKJ = await self._send_goal_to_client(self._action_client_LKJ, goal_msg, 'server_LKJ')
        # # result_client_RKJ = await self._send_goal_to_client(self._action_client_RKJ, goal_msg_inv, 'server_RKJ')
        # # result_client_LIKJ = await self._send_goal_to_client(self._action_client_LIKJ, goal_msg, 'server_LIKJ')
        # # result_client_RIKJ = await self._send_goal_to_client(self._action_client_RIKJ, goal_msg_inv, 'server_RIKJ')
        # # result_client_LFTJ = await self._send_goal_to_client(self._action_client_LFTJ, goal_msg_inv, 'server_LFTJ')
        # # result_client_RFTJ = await self._send_goal_to_client(self._action_client_RFTJ, goal_msg, 'server_RFTJ')
        # result_client_LKJ, result_client_RKJ, result_client_LIKJ, result_client_RIKJ, result_client_LFTJ, result_client_RFTJ = results
        # # Process the results and determine the outcome
        # if result_client_LKJ and result_client_RKJ and result_client_LIKJ and result_client_RIKJ and result_client_LFTJ and result_client_RFTJ:
        #     self.get_logger().info('All action clients completed successfully.')
        #     goal_handle.succeed()

        #     # Return the combined result
        #     result = GripperPosition.Result()
        #     result.position = result_client_LKJ.result.position
        #     result.effort = result_client_LKJ.result.effort
        #     result.stalled = result_client_LKJ.result.stalled
        #     result.reached_goal = result_client_LKJ.result.reached_goal and result_client_RKJ.result.reached_goal \
        #         and result_client_LIKJ.result.reached_goal and result_client_RIKJ.result.reached_goal \
        #         and result_client_LFTJ.result.reached_goal and result_client_RFTJ.result.reached_goal
        #     return result
        # else:
        #     self.get_logger().info('One or multiple action clients failed.')
        #     goal_handle.abort()

        #     result = GripperPosition.Result()
        #     result.position = result_client_LKJ.result.position
        #     result.effort = result_client_LKJ.result.effort
        #     result.stalled = result_client_LKJ.result.stalled
        #     result.reached_goal = result_client_LKJ.result.reached_goal and result_client_RKJ.result.reached_goal \
        #         and result_client_LIKJ.result.reached_goal and result_client_RIKJ.result.reached_goal \
        #         and result_client_LFTJ.result.reached_goal and result_client_RFTJ.result.reached_goal
        #     return result

    def _sleep_async(self, duration):
        # A simple async sleep helper function
        return self.create_timer(duration, lambda: None)
    
    ### Send goal to an action client ###
    async def _send_goal_to_client(self, client, goal_msg, server_name):
        client.wait_for_server()
        self.get_logger().info(f'Connected to {server_name}, sending goal...')

        send_goal_future = client.send_goal_async(goal_msg)
        goal_handle = await send_goal_future

        if not goal_handle.accepted:
            self.get_logger().info(f'Goal rejected by {server_name}')
            return None

        self.get_logger().info(f'Goal accepted by {server_name}, waiting for result...')
        result_future = goal_handle.get_result_async()
        result = await result_future
        
        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f'Result from {server_name}: {result.result.position}')
            return result
        else:
            self.get_logger().info(f'Goal failed on {server_name}')
            return None

    ### Wait for futures to complete ###
    async def _wait_for_futures(self, futures):
        results = []
        for future in futures:
            result = await future
            results.append(result)
        return results
    
    ### Goal handling ###
    def handle_goal(self, goal_handle):
        self.get_logger().info('Received goal request.')
        return GoalResponse.ACCEPT

    def handle_cancel(self, goal_handle):
        self.get_logger().info('Received cancel request.')
        return CancelResponse.ACCEPT

    def handle_accepted(self, goal_handle):
        self.get_logger().info('Goal accepted, starting execution thread...')
        goal_handle.execute()

    def joint_state_callback(self, msg):

        if not self.joint_state_msg_received:

            # check start state
            limit_exceeded = [False] * len(msg.name)
            for idx, enum in enumerate(msg.name):
                if enum == 'robotiq_85_left_knuckle_joint':
                    self.current_position = np.round(msg.position[idx], 2)
                    self.get_logger().info(f'Current gripper position is: {self.current_position}')

            self.joint_state_msg_received = True
        else:
            return


def main(args= None):
    rclpy.init(args=args)
    node = GripperPositionControl()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()