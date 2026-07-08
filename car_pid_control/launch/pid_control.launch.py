import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
	pkg_share = get_package_share_directory("car_pid_control")
	default_params = os.path.join(pkg_share, "config", "mission_params.yaml")

	params_arg = DeclareLaunchArgument(
		"params_file",
		default_value=default_params,
		description="PID 循迹节点的参数文件(目的点坐标、PID 增益、车辆参数等)",
	)

	pid_node = Node(
		package="car_pid_control",
		executable="pid_controller_node",
		name="pid_controller_node",
		output="screen",
		parameters=[LaunchConfiguration("params_file")],
	)

	return LaunchDescription([
		params_arg,
		pid_node,
	])
