import rclpy
from rclpy.node import Node
import yaml 
import os
import pathlib

from geometry_msgs.msg import  PointStamped
from nav_2d_msgs.msg import Pose2DStamped
import rclpy.parameter
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Directions


class PointSelector(Node):
    def __init__(self, node_name: str = 'tb4_point_selector', *, context: rclpy.Context | None = None, cli_args: rclpy.List[str] | None = None, namespace: str | None = None, use_global_arguments: bool = True, enable_rosout: bool = True, start_parameter_services: bool = True, parameter_overrides: rclpy.List[rclpy.Parameter] | None = None, allow_undeclared_parameters: bool = False, automatically_declare_parameters_from_overrides: bool = False) -> None:
        super().__init__(node_name, context=context, cli_args=cli_args, namespace=namespace, use_global_arguments=use_global_arguments, enable_rosout=enable_rosout, start_parameter_services=start_parameter_services, parameter_overrides=parameter_overrides, allow_undeclared_parameters=allow_undeclared_parameters, automatically_declare_parameters_from_overrides=automatically_declare_parameters_from_overrides)
        self._point_subscriber = self.create_subscription(PointStamped, 'clicked_point', self.clicked_point_callback, 10)
        self.get_logger().info('Point selector has started.')
        self.i = 0
        self.n_points = 0
        self.point_names = []
        self.point_values = []

    def clicked_point_callback(self, msg):
        c = input('Are you sure that you want to save this point? (y/N): ')
        if c == 'y':
            l = input('Want to name it? (y/N): ')
            if l == 'y':
                n = input('Enter the name: ')
                self.get_logger().info(f"Point name is: '{n}'")
            elif l == 'N':
                n = 'point_'+ str(self.i)
                self.get_logger().info(f"Generic point name is: '{n}'")
                self.i += 1
            h = input('In which direction? (N/E/S/W): ')
            if h == 'N':
                orientation = TurtleBot4Directions.NORTH
                self.get_logger().info(f"North direction was selected ({TurtleBot4Directions.NORTH})")
            elif h == 'E':
                orientation = TurtleBot4Directions.EAST
                self.get_logger().info(f"East direction was selected ({TurtleBot4Directions.EAST})")
            elif h == 'S':
                orientation = TurtleBot4Directions.SOUTH
                self.get_logger().info(f"South direction was selected ({TurtleBot4Directions.SOUTH})")
            elif h == 'W':
                orientation = TurtleBot4Directions.WEST
                self.get_logger().info(f"West direction was selected ({TurtleBot4Directions.WEST})")
            else:
                orientation = 0
            
            self.point_names.append(n)
            self.point_values += (msg.point.x, msg.point.y, float(orientation))
            self.n_points += 1

            self.get_logger().info(f"Saved: ({orientation}, {msg.point.x}, {msg.point.y})")

            q = input('Keep recording? (y/N): ')
            if q == 'N':
                self.create_yaml_file()                
                raise SystemExit           # <--- here is we exit the node
        elif c == 'N':
            if self.n_points > 0 :
                self.create_yaml_file()                
                raise SystemExit           # <--- here is we exit the node
            else:
                raise SystemExit           # <--- here is we exit the node

    def end_node(self):
        rclpy.shutdown()
          

    def generate_msg(self, name, x, y, theta):
        pose = Pose2DStamped()
        pose.header.stamp.sec, pose.header.stamp.nanosec = self.get_clock().now().seconds_nanoseconds()
        pose.header.frame_id = name
        pose.pose.x = x
        pose.pose.y = y
        pose.pose.theta = float(theta)
        
        return pose

    def create_yaml_file(self):
        self.get_logger().info(f"Writing YAML file and shutdown")
        data = dict(basic_navigator=dict(ros__parameters=dict(no_points=self.n_points, names=self.point_names, values=self.point_values)))
        # dict_file = [{'node_name': {'no_points':self.n_points, 'names':self.point_names, 'values':self.point_values}}]
        path = os.path.join(pathlib.Path().home(), 'Workspaces/mcr_course_ws/src/class_pkg/config', 'params.yaml')
        with open(path, 'w') as file:
            documents = yaml.dump(data, file, default_flow_style=False)
        self.get_logger().info(f"File created.")

def main(args=None):
    rclpy.init(args=args)
    node = PointSelector()
    try:
        rclpy.spin(node)
    except SystemExit:                 # <--- process the exception 
        rclpy.logging.get_logger("Quitting").info('Done')
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()