#pragma once

namespace car_pid_control
{
// 通用的一维 PID 控制器，带积分抗饱和与输出限幅。
// 供 pid_controller_node 用于「距离环」和「航向环」两路独立控制。
class Pid
{
public:
	Pid() = default;
	Pid(double kp, double ki, double kd,
		double out_min, double out_max,
		double i_min, double i_max);

	void set_gains(double kp, double ki, double kd);
	void set_output_limits(double out_min, double out_max);
	void set_integral_limits(double i_min, double i_max);

	// 输入误差 error 与时间步长 dt(秒),返回限幅后的控制量。
	double compute(double error, double dt);

	// 清零积分与历史误差,切换目标点/停车时调用,避免残留积分。
	void reset();

private:
	double clamp(double value, double lo, double hi) const;

	double kp_{0.0};
	double ki_{0.0};
	double kd_{0.0};

	double out_min_{-1.0};
	double out_max_{1.0};
	double i_min_{-1.0};
	double i_max_{1.0};

	double integral_{0.0};
	double prev_error_{0.0};
	bool has_prev_{false};
};
}  // namespace car_pid_control
