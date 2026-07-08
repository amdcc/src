#include "uart_to_mcu/uart_to_mcu.hpp"

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fcntl.h>
#include <limits>
#include <termios.h>
#include <unistd.h>

namespace uart_to_mcu
{
UartToMcuNode::UartToMcuNode()
: Node("uart_to_mcu_node"), serial_fd_(-1)
{
	wheel_speeds_topic_ =
		declare_parameter<std::string>("wheel_speeds_topic", kDefaultWheelSpeedsTopic);
	goal_reached_topic_ =
		declare_parameter<std::string>("goal_reached_topic", kDefaultGoalReachedTopic);
	goal_published_topic_ =
		declare_parameter<std::string>("goal_published_topic", kDefaultGoalPublishedTopic);
	serial_port_ = declare_parameter<std::string>("serial_port", "/dev/ttyS1");
	// /wheel_speeds 单位为 RPS(转/秒),乘以该系数换算成下位机期望的整数车速(编码器 cps 等)。
	wheel_speed_scale_ = declare_parameter<double>("wheel_speed_scale", 15.0);

	// 行进包:订阅左右轮速,持续下发(id=0x37)。
	wheel_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
		wheel_speeds_topic_, 20,
		std::bind(&UartToMcuNode::wheel_speeds_callback, this, std::placeholders::_1));
	// 开始包:收到新目标发布事件时启动小车(id=0x34)。
	goal_published_sub_ = create_subscription<std_msgs::msg::Bool>(
		goal_published_topic_, 10,
		std::bind(&UartToMcuNode::goal_published_callback, this, std::placeholders::_1));
	// 结束包:到达目标或停车时下发停止(id=0x38)。
	goal_reached_sub_ = create_subscription<std_msgs::msg::Bool>(
		goal_reached_topic_, 10,
		std::bind(&UartToMcuNode::goal_reached_callback, this, std::placeholders::_1));

	if (!open_serial()) {
		RCLCPP_ERROR(get_logger(), "Failed to open serial port: %s", serial_port_.c_str());
	} else {
		RCLCPP_INFO(get_logger(), "Serial port opened: %s (115200,8N1)", serial_port_.c_str());
	}
}

UartToMcuNode::~UartToMcuNode()
{
	close_serial();
}

bool UartToMcuNode::open_serial()
{
	close_serial();

	serial_fd_ = ::open(serial_port_.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
	if (serial_fd_ < 0) {
		RCLCPP_ERROR(get_logger(), "open(%s) failed: %s", serial_port_.c_str(), std::strerror(errno));
		return false;
	}

	termios tty{};
	if (tcgetattr(serial_fd_, &tty) != 0) {
		RCLCPP_ERROR(get_logger(), "tcgetattr failed: %s", std::strerror(errno));
		close_serial();
		return false;
	}

	cfsetospeed(&tty, B115200);
	cfsetispeed(&tty, B115200);

	tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;  // 8 data bits
	tty.c_cflag |= CLOCAL | CREAD;
	tty.c_cflag &= ~(PARENB | PARODD);            // no parity
	tty.c_cflag &= ~CSTOPB;                       // 1 stop bit
	tty.c_cflag &= ~CRTSCTS;                      // no HW flow control

	tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
	tty.c_lflag = 0;
	tty.c_oflag = 0;
	tty.c_cc[VMIN] = 0;
	tty.c_cc[VTIME] = 1;

	if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) {
		RCLCPP_ERROR(get_logger(), "tcsetattr failed: %s", std::strerror(errno));
		close_serial();
		return false;
	}

	return true;
}

void UartToMcuNode::close_serial()
{
	if (serial_fd_ >= 0) {
		::close(serial_fd_);
		serial_fd_ = -1;
	}
}

int32_t UartToMcuNode::scale_to_i32(double rps) const
{
	const double scaled = std::round(rps * wheel_speed_scale_);
	const double min_v = static_cast<double>(std::numeric_limits<int32_t>::min());
	const double max_v = static_cast<double>(std::numeric_limits<int32_t>::max());
	if (scaled < min_v) {
		return std::numeric_limits<int32_t>::min();
	}
	if (scaled > max_v) {
		return std::numeric_limits<int32_t>::max();
	}
	return static_cast<int32_t>(scaled);
}

void UartToMcuNode::wheel_speeds_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
	if (msg->data.size() < 2) {
		RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
			"wheel_speeds message needs at least 2 values");
		return;
	}

	const int32_t left = scale_to_i32(msg->data[0]);
	const int32_t right = scale_to_i32(msg->data[1]);

	// 行进包:下发左右轮车速。
	if (!write_packet(build_packet(kMovePacketId, left, right))) {
		RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Failed writing move packet to serial");
	}
}

void UartToMcuNode::goal_published_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
	if (!msg->data) {
		return;
	}

	// 开始包:启动小车,左右轮速填 0。
	if (!write_packet(build_packet(kStartPacketId, 0, 0))) {
		RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Failed writing start packet to serial");
		return;
	}
	RCLCPP_INFO(get_logger(), "Sent start packet (id=0x%02X)", kStartPacketId);
}

void UartToMcuNode::goal_reached_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
	if (!msg->data) {
		return;
	}

	// 结束包:停止行驶,左右轮速填 0。
	if (!write_packet(build_packet(kEndPacketId, 0, 0))) {
		RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Failed writing end packet to serial");
		return;
	}
	RCLCPP_INFO(get_logger(), "Sent end packet (id=0x%02X)", kEndPacketId);
}

std::vector<uint8_t> UartToMcuNode::build_packet(
	uint16_t uart_id, int32_t speed_left, int32_t speed_right) const
{
	std::vector<uint8_t> packet;
	packet.reserve(2 + 2 + 4 + 4 + 2);

	// 帧头 0x55AA:先 0x55 再 0xAA。
	packet.push_back(0x55U);
	packet.push_back(0xAAU);
	append_u16_le(packet, uart_id);
	append_i32_le(packet, speed_left);
	append_i32_le(packet, speed_right);
	// 帧尾 0xAA55:先 0xAA 再 0x55。
	packet.push_back(0xAAU);
	packet.push_back(0x55U);

	return packet;
}

bool UartToMcuNode::write_packet(const std::vector<uint8_t> & packet)
{
	if (serial_fd_ < 0 && !open_serial()) {
		return false;
	}

	size_t total_written = 0;
	while (total_written < packet.size()) {
		const ssize_t written =
			::write(serial_fd_, packet.data() + total_written, packet.size() - total_written);
		if (written < 0) {
			if (errno == EINTR) {
				continue;
			}
			RCLCPP_ERROR(get_logger(), "write failed: %s", std::strerror(errno));
			close_serial();
			return false;
		}
		total_written += static_cast<size_t>(written);
	}
	return true;
}

void UartToMcuNode::append_u16_le(std::vector<uint8_t> & out, uint16_t value)
{
	out.push_back(static_cast<uint8_t>(value & 0xFF));
	out.push_back(static_cast<uint8_t>((value >> 8) & 0xFF));
}

void UartToMcuNode::append_i32_le(std::vector<uint8_t> & out, int32_t value)
{
	const auto u = static_cast<uint32_t>(value);
	out.push_back(static_cast<uint8_t>(u & 0xFF));
	out.push_back(static_cast<uint8_t>((u >> 8) & 0xFF));
	out.push_back(static_cast<uint8_t>((u >> 16) & 0xFF));
	out.push_back(static_cast<uint8_t>((u >> 24) & 0xFF));
}
}  // namespace uart_to_mcu

int main(int argc, char ** argv)
{
	rclcpp::init(argc, argv);
	auto node = std::make_shared<uart_to_mcu::UartToMcuNode>();
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}
