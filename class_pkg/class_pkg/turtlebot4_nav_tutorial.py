from math import floor
from threading import Lock, Thread
from time import sleep

import rclpy

from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import BatteryState
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Directions, TurtleBot4Navigator

BATTERY_HIGH = 0.95
BATTERY_LOW = 0.2  # when the robot will go charge
BATTERY_CRITICAL = 0.1  # when the robot will shutdown

class BatteryMonitor(Node):

    def __init__(self, lock):
        super().__init__('battery_monitor')
        self.get_logger().info("Initializing battery monitor node with namespace: " + self.get_namespace())
        self.lock = lock

        # Subscribe to the /battery_state topic
        self.battery_state_subscriber = self.create_subscription(
            BatteryState,
            'battery_state',
            self.battery_state_callback,
            qos_profile_sensor_data)
        
        self.get_logger().info("Battery monitor listening to: " + self.battery_state_subscriber.topic_name)

    # Callbacks
    def battery_state_callback(self, batt_msg: BatteryState):
        with self.lock:
            self.battery_percent = batt_msg.percentage

    def thread_function(self):
        executor = SingleThreadedExecutor()
        executor.add_node(self)
        executor.spin()


# Define a function to split a list into chunks of a specified size
def split_list(lst, chunk_size):
    # Use a list comprehension to create chunks
    # For each index 'i' in the range from 0 to the length of the list with step 'chunk_size'
    # Slice the list from index 'i' to 'i + chunk_size'
    return [lst[i:i + chunk_size] for i in range(0, len(lst), chunk_size)]

def main(args=None):
    rclpy.init(args=args)

    lock = Lock()
    battery_monitor = BatteryMonitor(lock)

    navigator = TurtleBot4Navigator()
    navigator.declare_parameters(
        namespace='',
        parameters=[
            ('names', rclpy.Parameter.Type.STRING_ARRAY),
            ('no_points', rclpy.Parameter.Type.INTEGER),
            ('values', rclpy.Parameter.Type.DOUBLE_ARRAY)
        ]
    )
    battery_percent = None
    position_index = 0

    thread = Thread(target=battery_monitor.thread_function, daemon=True)
    thread.start()

    navigator.info('Initializing navigation patrol node. Namespace: ' + navigator.get_namespace())
    
    # Start on dock
    if not navigator.getDockedStatus():
        navigator.info('Docking before intialising pose')
        navigator.dock()

    # Set initial pose
    initial_pose = navigator.getPoseStamped([0.0, 0.0], TurtleBot4Directions.NORTH)
    navigator.setInitialPose(initial_pose)

    # Wait for Nav2
    navigator.waitUntilNav2Active()

    # Undock
    navigator.undock()

    # Get parameters
    names, no_points, values = navigator.get_parameters(['names', 'no_points', 'values'])
    split_values = split_list(values.value, 3)
    points = dict(zip(names.value, split_values))

    navigator.info(f'No. points: {no_points.value}')

    # Prepare goal poses
    goal_pose = []
    for pose in split_values:
        goal_pose.append(navigator.getPoseStamped(pose[:-1], pose[-1]))
    # goal_pose.append(navigator.getPoseStamped([-1.0, -0.22], TurtleBot4Directions.EAST))
    # goal_pose.append(navigator.getPoseStamped([-0.77, -1.27], TurtleBot4Directions.NORTH))
    # goal_pose.append(navigator.getPoseStamped([-2.06, -0.35], TurtleBot4Directions.NORTH_WEST))
    # goal_pose.append(navigator.getPoseStamped([-1.025, 1.025], TurtleBot4Directions.WEST))

    while True:
        with lock:
            battery_percent = battery_monitor.battery_percent

        if (battery_percent is not None):
            navigator.info(f'Battery is at {(battery_percent*100):.2f}% charge')

            # Check battery charge level
            if (battery_percent < BATTERY_CRITICAL):
                navigator.error('Battery critically low. Charge or power down')
                break
            elif (battery_percent < BATTERY_LOW):
                # Go near the dock
                navigator.info('Docking for charge')
                near_dock = points['near_dock']
                navigator.startToPose(navigator.getPoseStamped(near_dock[:-1],
                                      near_dock[:-1]))
                navigator.dock()

                if not navigator.getDockedStatus():
                    navigator.error('Robot failed to dock')
                    break

                # Wait until charged
                navigator.info('Charging...')
                battery_percent_prev = 0
                while (battery_percent < BATTERY_HIGH):
                    sleep(15)
                    battery_percent_prev = floor(battery_percent*100)/100
                    with lock:
                        battery_percent = battery_monitor.battery_percent

                    # Print charge level every time it increases a percent
                    if battery_percent > (battery_percent_prev + 0.01):
                        navigator.info(f'Battery is at {(battery_percent*100):.2f}% charge')

                # Undock
                navigator.undock()
                position_index = 0

            else:
                # Navigate to next position
                navigator.startToPose(goal_pose[position_index])
                # navigator.info(f'Doing nothing')
                position_index = position_index + 1
                if position_index >= len(goal_pose):
                    position_index = 0

    battery_monitor.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()