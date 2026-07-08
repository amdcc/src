#include "car_pid_control/pid_controller_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <vector>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/exceptions.h"
#include "tf2/time.h"

namespace car_pid_control
{
PidControllerNode::PidControllerNode()
: Node("pid_controller_node")
{
	// ---- 话题 / 坐标系参数 ----
	map_frame_ = declare_parameter<std::string>("map_frame", "map");
	base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
	mission_topic_ = declare_parameter<std::string>("mission_topic", "/car_mission_start");
	wheel_speeds_topic_ = declare_parameter<std::string>("wheel_speeds_topic", "/wheel_speeds");
	goal_published_topic_ = declare_parameter<std::string>("goal_published_topic", "/goal_published");
	goal_reached_topic_ = declare_parameter<std::string>("goal_reached_topic", "/goal_reached");

	// ---- 运动 / 控制参数 ----
	control_frequency_ = declare_parameter<double>("control_frequency", 50.0);
	wheel_base_ = declare_parameter<double>("wheel_base", 0.20);
	wheel_radius_ = declare_parameter<double>("wheel_radius", 0.0325);
	max_linear_speed_ = declare_parameter<double>("max_linear_speed", 0.4);
	max_angular_speed_ = declare_parameter<double>("max_angular_speed", 1.5);
	max_wheel_rps_ = declare_parameter<double>("max_wheel_rps", 6.0);
	goal_tolerance_ = declare_parameter<double>("goal_tolerance", 0.08);
	heading_gate_ = declare_parameter<double>("heading_gate", 0.6);

	// ---- PID 增益 ----
	const double lin_kp = declare_parameter<double>("linear_kp", 0.8);
	const double lin_ki = declare_parameter<double>("linear_ki", 0.0);
	const double lin_kd = declare_parameter<double>("linear_kd", 0.05);
	const double ang_kp = declare_parameter<double>("angular_kp", 1.8);
	const double ang_ki = declare_parameter<double>("angular_ki", 0.0);
	const double ang_kd = declare_parameter<double>("angular_kd", 0.10);

	linear_pid_.set_gains(lin_kp, lin_ki, lin_kd);
	linear_pid_.set_output_limits(0.0, max_linear_speed_);  // 距离环只前进,不倒车
	linear_pid_.set_integral_limits(-max_linear_speed_, max_linear_speed_);
	angular_pid_.set_gains(ang_kp, ang_ki, ang_kd);
	angular_pid_.set_output_limits(-max_angular_speed_, max_angular_speed_);
	angular_pid_.set_integral_limits(-max_angular_speed_, max_angular_speed_);

	// ---- 三个目的点 A / B / C(map 坐标系,单位米)----
	const auto goal_a = declare_parameter<std::vector<double>>("goal_a", {2.0, 0.0});
	const auto goal_b = declare_parameter<std::vector<double>>("goal_b", {2.0, 2.0});
	const auto goal_c = declare_parameter<std::vector<double>>("goal_c", {0.0, 2.0});
	const auto load_wp = [this](const std::vector<double> & v, const char * name) {
		if (v.size() < 2) {
			RCLCPP_WARN(get_logger(), "%s 需要 [x, y] 两个值,已回退到 (0,0)", name);
			return Waypoint{0.0, 0.0};
		}
		return Waypoint{v[0], v[1]};
	};
	waypoints_[0] = load_wp(goal_a, "goal_a");
	waypoints_[1] = load_wp(goal_b, "goal_b");
	waypoints_[2] = load_wp(goal_c, "goal_c");

	// ---- TF ----
	tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
	tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

	// ---- 发布 / 订阅 ----
	wheel_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(wheel_speeds_topic_, 20);
	goal_published_pub_ = create_publisher<std_msgs::msg::Bool>(goal_published_topic_, 10);
	goal_reached_pub_ = create_publisher<std_msgs::msg::Bool>(goal_reached_topic_, 10);
	mission_sub_ = create_subscription<std_msgs::msg::String>(
		mission_topic_, 10,
		std::bind(&PidControllerNode::mission_callback, this, std::placeholders::_1));

	const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, control_frequency_));
	control_timer_ = create_wall_timer(
		std::chrono::duration_cast<std::chrono::nanoseconds>(period),
		std::bind(&PidControllerNode::control_loop, this));

	RCLCPP_INFO(get_logger(),
		"pid_controller_node 已启动: 监听 %s (1->A 2->B 3->C 0->停车), map=%s base=%s",
		mission_topic_.c_str(), map_frame_.c_str(), base_frame_.c_str());
}

void PidControllerNode::mission_callback(const std_msgs::msg::String::SharedPtr msg)
{
	// 去除首尾空白,只取指令字符。
	std::string cmd = msg->data;
	cmd.erase(0, cmd.find_first_not_of(" \t\r\n"));
	const auto last = cmd.find_last_not_of(" \t\r\n");
	if (last != std::string::npos) {
		cmd.erase(last + 1);
	}

	int value = -1;
	try {
		value = std::stoi(cmd);
	} catch (const std::exception &) {
		RCLCPP_WARN(get_logger(), "收到无法解析的指令: '%s' (期望 0/1/2/3)", msg->data.c_str());
		return;
	}

	if (value == 0) {
		state_ = MissionState::kStop;
		active_goal_ = 0;
		linear_pid_.reset();
		angular_pid_.reset();
		publish_stop();
		RCLCPP_INFO(get_logger(), "收到指令 0: 停车");
		return;
	}

	if (value < 1 || value > static_cast<int>(waypoints_.size())) {
		RCLCPP_WARN(get_logger(), "指令 %d 超出目的点范围 (1~%zu)", value, waypoints_.size());
		return;
	}

	active_goal_ = value;
	state_ = MissionState::kNavigate;
	linear_pid_.reset();
	angular_pid_.reset();

	// 通知串口节点:有新目标发布(首次会触发下位机启动包)。
	std_msgs::msg::Bool published;
	published.data = true;
	goal_published_pub_->publish(published);

	const Waypoint & wp = waypoints_[static_cast<size_t>(value - 1)];
	RCLCPP_INFO(get_logger(), "收到指令 %d: 前往目的点 %c = (%.2f, %.2f)",
		value, static_cast<char>('A' + value - 1), wp.x, wp.y);
}

void PidControllerNode::control_loop()
{
	const rclcpp::Time now = get_clock()->now();
	double dt = 1.0 / std::max(1.0, control_frequency_);
	if (has_last_time_) {
		const double measured = (now - last_loop_time_).seconds();
		if (measured > 0.0 && measured < 1.0) {
			dt = measured;
		}
	}
	last_loop_time_ = now;
	has_last_time_ = true;

	if (state_ != MissionState::kNavigate) {
		publish_stop();
		return;
	}

	double x = 0.0;
	double y = 0.0;
	double yaw = 0.0;
	if (!get_current_pose(x, y, yaw)) {
		// 拿不到位姿时保持停车,避免盲跑。
		publish_stop();
		return;
	}

	const Waypoint & goal = waypoints_[static_cast<size_t>(active_goal_ - 1)];
	const double dx = goal.x - x;
	const double dy = goal.y - y;
	const double distance = std::hypot(dx, dy);

	// 已到达:停车并上报。
	if (distance <= goal_tolerance_) {
		state_ = MissionState::kReached;
		linear_pid_.reset();
		angular_pid_.reset();
		publish_stop();

		std_msgs::msg::Bool reached;
		reached.data = true;
		goal_reached_pub_->publish(reached);
		RCLCPP_INFO(get_logger(), "已到达目的点 %c,距离 %.3f m",
			static_cast<char>('A' + active_goal_ - 1), distance);
		return;
	}

	// 航向误差(目标方向 - 当前朝向),规整到 [-pi, pi]。
	const double target_heading = std::atan2(dy, dx);
	const double heading_err = normalize_angle(target_heading - yaw);

	const double w = angular_pid_.compute(heading_err, dt);

	double v = 0.0;
	if (std::fabs(heading_err) < heading_gate_) {
		// 基本对准目标后才前进,并按航向误差的余弦衰减线速度(转弯时减速)。
		v = linear_pid_.compute(distance, dt) * std::cos(heading_err);
		v = std::max(0.0, v);
	} else {
		// 航向误差过大,原地转向对准。
		linear_pid_.reset();
	}

	// 差速运动学:v_wheel = v ± w * (轮距 / 2)
	const double v_left = v - w * (wheel_base_ / 2.0);
	const double v_right = v + w * (wheel_base_ / 2.0);

	// 线速度(m/s)-> 车轮转速(转/秒): rps = v / (2*pi*r)
	const double circumference = 2.0 * M_PI * wheel_radius_;
	double left_rps = (circumference > 1e-6) ? v_left / circumference : 0.0;
	double right_rps = (circumference > 1e-6) ? v_right / circumference : 0.0;

	left_rps = clamp(left_rps, -max_wheel_rps_, max_wheel_rps_);
	right_rps = clamp(right_rps, -max_wheel_rps_, max_wheel_rps_);

	publish_wheel_speeds(left_rps, right_rps);
}

bool PidControllerNode::get_current_pose(double & x, double & y, double & yaw)
{
	geometry_msgs::msg::TransformStamped tf;
	try {
		// 用最新可用的 TF(时间戳 0),容忍传感器/建图的轻微延迟。
		tf = tf_buffer_->lookupTransform(map_frame_, base_frame_, tf2::TimePointZero);
	} catch (const tf2::TransformException & ex) {
		RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
			"无法获取 %s->%s 的 TF: %s", map_frame_.c_str(), base_frame_.c_str(), ex.what());
		return false;
	}

	x = tf.transform.translation.x;
	y = tf.transform.translation.y;

	const auto & q = tf.transform.rotation;
	// 由四元数计算平面偏航角 yaw。
	const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
	const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
	yaw = std::atan2(siny_cosp, cosy_cosp);
	return true;
}

void PidControllerNode::publish_wheel_speeds(double left_rps, double right_rps)
{
	std_msgs::msg::Float64MultiArray msg;
	msg.data = {left_rps, right_rps};
	wheel_pub_->publish(msg);
}

void PidControllerNode::publish_stop()
{
	publish_wheel_speeds(0.0, 0.0);
}

double PidControllerNode::normalize_angle(double angle)
{
	while (angle > M_PI) {
		angle -= 2.0 * M_PI;
	}
	while (angle < -M_PI) {
		angle += 2.0 * M_PI;
	}
	return angle;
}

double PidControllerNode::clamp(double value, double lo, double hi)
{
	return std::max(lo, std::min(value, hi));
}
}  // namespace car_pid_control

int main(int argc, char ** argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<car_pid_control::PidControllerNode>());
	rclcpp::shutdown();
	return 0;
}
