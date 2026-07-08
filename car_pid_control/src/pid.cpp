#include "car_pid_control/pid.hpp"

#include <algorithm>

namespace car_pid_control
{
Pid::Pid(double kp, double ki, double kd,
	double out_min, double out_max,
	double i_min, double i_max)
: kp_(kp), ki_(ki), kd_(kd),
	out_min_(out_min), out_max_(out_max),
	i_min_(i_min), i_max_(i_max)
{
}

void Pid::set_gains(double kp, double ki, double kd)
{
	kp_ = kp;
	ki_ = ki;
	kd_ = kd;
}

void Pid::set_output_limits(double out_min, double out_max)
{
	out_min_ = out_min;
	out_max_ = out_max;
}

void Pid::set_integral_limits(double i_min, double i_max)
{
	i_min_ = i_min;
	i_max_ = i_max;
}

double Pid::clamp(double value, double lo, double hi) const
{
	return std::max(lo, std::min(value, hi));
}

double Pid::compute(double error, double dt)
{
	if (dt <= 0.0) {
		// 非法时间步,只用比例项,避免除零/积分爆炸。
		return clamp(kp_ * error, out_min_, out_max_);
	}

	// 积分项 + 抗饱和(clamp integral)。
	integral_ += error * dt;
	integral_ = clamp(integral_, i_min_, i_max_);

	// 微分项(基于误差变化率),首个采样点不做微分。
	double derivative = 0.0;
	if (has_prev_) {
		derivative = (error - prev_error_) / dt;
	}
	prev_error_ = error;
	has_prev_ = true;

	const double output = kp_ * error + ki_ * integral_ + kd_ * derivative;
	return clamp(output, out_min_, out_max_);
}

void Pid::reset()
{
	integral_ = 0.0;
	prev_error_ = 0.0;
	has_prev_ = false;
}
}  // namespace car_pid_control
