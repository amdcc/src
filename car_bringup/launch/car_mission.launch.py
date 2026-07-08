import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
	"""小车任务控制整体 bring-up:PID 循迹节点 + 串口下发节点一键拉起。

	参数默认复用 car_pid_control 包内的 mission_params.yaml,可用 params_file 覆盖。

	注意:本 launch 不启动 cartographer,请先单独启动 cartographer 建图/定位,
	确保 map->base_link 的 TF 正常发布后再运行本 launch。
	"""
	# 参数文件默认取自 car_pid_control 包(配置的唯一来源,避免重复维护)。
	pid_pkg_share = get_package_share_directory("car_pid_control")
	default_params = os.path.join(pid_pkg_share, "config", "mission_params.yaml")

	params_arg = DeclareLaunchArgument(
		"params_file",
		default_value=default_params,
		description="PID 循迹节点的参数文件(默认为 car_pid_control/config/mission_params.yaml)",
	)
	serial_port_arg = DeclareLaunchArgument(
		"serial_port",
		default_value="/dev/ttyS1",
		description="下发轮速数据包的串口一(RDK X5 GPIO UART1: 引脚 8/10)",
	)
	wheel_scale_arg = DeclareLaunchArgument(
		"wheel_speed_scale",
		default_value="5.8",
		description="轮速(RPS)下发到下位机前的缩放系数",
	)

	# PID 循迹 / 任务控制节点:订阅 /car_mission_start,输出 /wheel_speeds
	pid_node = Node(
		package="car_pid_control",
		executable="pid_controller_node",
		name="pid_controller_node",
		output="screen",
		parameters=[LaunchConfiguration("params_file")],
	)

	# 串口节点:订阅 /wheel_speeds、/goal_published、/goal_reached,打包发给下位机
	uart_node = Node(
		package="uart_to_mcu",
		executable="uart_to_mcu_node",
		name="uart_to_mcu_node",
		output="screen",
		parameters=[{
			"serial_port": LaunchConfiguration("serial_port"),
			"wheel_speeds_topic": "/wheel_speeds",
			"wheel_speed_scale": LaunchConfiguration("wheel_speed_scale"),
		}],
	)

	return LaunchDescription([
		params_arg,
		serial_port_arg,
		wheel_scale_arg,
		pid_node,
		uart_node,
	])
