#pragma once

#include <array>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "car_pid_control/pid.hpp"

namespace car_pid_control
{
// 一个目的点(A/B/C)的坐标,单位米,坐标系为 cartographer 的 map。
struct Waypoint
{
	double x{0.0};
	double y{0.0};
};

// 小车任务状态机。
enum class MissionState
{
	kStop,      // 停车等待(收到 "0" 或尚未收到任务)
	kNavigate,  // 正在前往目的点
	kReached    // 已到达,保持停止直到下一条指令
};

// 通过 /car_mission_start 接收任务指令,基于 cartographer 位姿(map->base_link 的 TF)
// 做「距离 + 航向」双 PID,输出左右轮转速到 /wheel_speeds,交给串口节点下发给下位机。
class PidControllerNode : public rclcpp::Node
{
public:
	PidControllerNode();

private:
	void mission_callback(const std_msgs::msg::String::SharedPtr msg);
	void control_loop();

	// 取当前 map->base_link 位姿,成功返回 true。
	bool get_current_pose(double & x, double & y, double & yaw);

	// 向 /wheel_speeds 发布左右轮转速(单位 RPS,转/秒)。
	void publish_wheel_speeds(double left_rps, double right_rps);
	void publish_stop();

	static double normalize_angle(double angle);
	static double clamp(double value, double lo, double hi);

	// ---- 参数 ----
	std::string map_frame_;
	std::string base_frame_;
	std::string mission_topic_;
	std::string wheel_speeds_topic_;
	std::string goal_published_topic_;
	std::string goal_reached_topic_;

	double control_frequency_{50.0};
	double wheel_base_{0.20};     // 左右轮间距(轮距),米
	double wheel_radius_{0.0325}; // 车轮半径,米
	double max_linear_speed_{0.4};   // m/s
	double max_angular_speed_{1.5};  // rad/s
	double max_wheel_rps_{6.0};      // 单轮最大转速,转/秒
	double goal_tolerance_{0.08};    // 到点距离阈值,米
	double heading_gate_{0.6};       // 航向误差大于该值时原地转向,弧度

	std::array<Waypoint, 3> waypoints_{};  // 索引 0/1/2 -> A/B/C

	// ---- 控制器 ----
	Pid linear_pid_;
	Pid angular_pid_;

	// ---- 状态 ----
	MissionState state_{MissionState::kStop};
	int active_goal_{0};  // 0=无, 1/2/3 对应 A/B/C
	rclcpp::Time last_loop_time_;
	bool has_last_time_{false};

	// ---- ROS 接口 ----
	rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mission_sub_;
	rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr wheel_pub_;
	rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr goal_published_pub_;
	rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr goal_reached_pub_;
	rclcpp::TimerBase::SharedPtr control_timer_;

	std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
	std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};
}  // namespace car_pid_control
