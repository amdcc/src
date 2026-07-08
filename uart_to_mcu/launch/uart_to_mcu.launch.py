from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
	serial_port_arg = DeclareLaunchArgument(
		"serial_port",
		default_value="/dev/ttyS1",
		description="Serial device for sending wheel speed packets (RDK X5 GPIO UART1: pins 8/10)",
	)

	wheel_topic_arg = DeclareLaunchArgument(
		"wheel_speeds_topic",
		default_value="/wheel_speeds",
		description="Topic that publishes left/right wheel speeds",
	)

	wheel_speed_scale_arg = DeclareLaunchArgument(
		"wheel_speed_scale",
		default_value="5.8",
		description="Multiplier applied to RPS values before sending to MCU (tune to match MCU PWM range)",
	)

	uart_node = Node(
		package="uart_to_mcu",
		executable="uart_to_mcu_node",
		name="uart_to_mcu_node",
		output="screen",
		parameters=[{
			"serial_port": LaunchConfiguration("serial_port"),
			"wheel_speeds_topic": LaunchConfiguration("wheel_speeds_topic"),
			"wheel_speed_scale": LaunchConfiguration("wheel_speed_scale"),
		}],
	)

	return LaunchDescription([
		serial_port_arg,
		wheel_topic_arg,
		wheel_speed_scale_arg,
		uart_node,
	])
