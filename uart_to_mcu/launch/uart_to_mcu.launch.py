from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
	"""单独启动串口下发节点(便于脱离循迹端单测串口)。"""
	serial_port_arg = DeclareLaunchArgument(
		"serial_port",
		default_value="/dev/ttyS1",
		description="下发数据包的串口(RDK X5 GPIO UART1: 引脚 8/10)",
	)
	wheel_scale_arg = DeclareLaunchArgument(
		"wheel_speed_scale",
		default_value="15.0",
		description="行进包中 RPS -> 下位机整数车速的缩放系数",
	)

	uart_node = Node(
		package="uart_to_mcu",
		executable="uart_to_mcu_node",
		name="uart_to_mcu_node",
		output="screen",
		parameters=[{
			"serial_port": LaunchConfiguration("serial_port"),
			"wheel_speed_scale": LaunchConfiguration("wheel_speed_scale"),
		}],
	)

	return LaunchDescription([
		serial_port_arg,
		wheel_scale_arg,
		uart_node,
	])
