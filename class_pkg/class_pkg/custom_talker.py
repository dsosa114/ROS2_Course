import rclpy
from rclpy.node import Node

class CustomTalker(Node):
    def __init__(self, node_name: str = "custom_talker", *, context: rclpy.Context | None = None, cli_args: rclpy.List[str] | None = None, namespace: str | None = None, use_global_arguments: bool = True, enable_rosout: bool = True, start_parameter_services: bool = True, parameter_overrides: rclpy.List[rclpy.Parameter] | None = None, allow_undeclared_parameters: bool = False, automatically_declare_parameters_from_overrides: bool = False) -> None:
        super().__init__(node_name, context=context, cli_args=cli_args, namespace=namespace, use_global_arguments=use_global_arguments, enable_rosout=enable_rosout, start_parameter_services=start_parameter_services, parameter_overrides=parameter_overrides, allow_undeclared_parameters=allow_undeclared_parameters, automatically_declare_parameters_from_overrides=automatically_declare_parameters_from_overrides)
        self.get_logger().info('Hi from class_pkg.')

def main(args=None):
    rclpy.init(args=args)
    node = CustomTalker()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
