#include "uart_to_mcu/uart_to_mcu.hpp"

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fcntl.h>
#include <limits>
#include <termios.h>
#include <type_traits>
#include <unistd.h>

namespace uart_to_mcu
{
UartToMcuNode::UartToMcuNode()
: Node("uart_to_mcu_node"), serial_fd_(-1), start_flag_(true)
{
	wheel_speeds_topic_ =
		declare_parameter<std::string>("wheel_speeds_topic", kDefaultWheelSpeedsTopic);
	goal_reached_topic_ =
		declare_parameter<std::string>("goal_reached_topic", kDefaultGoalReachedTopic);
	goal_published_topic_ =
		declare_parameter<std::string>("goal_published_topic", kDefaultGoalPublishedTopic);
	serial_port_ = declare_parameter<std::string>("serial_port", "/dev/ttyS1");
	wheel_speed_scale_ = declare_parameter<double>("wheel_speed_scale", 5.8);

	wheel_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
		wheel_speeds_topic_, 20, std::bind(&UartToMcuNode::wheel_speeds_callback, this, std::placeholders::_1));
	goal_reached_sub_ = create_subscription<std_msgs::msg::Bool>(
		goal_reached_topic_, 10, std::bind(&UartToMcuNode::goal_reached_callback, this, std::placeholders::_1));
	goal_published_sub_ = create_subscription<std_msgs::msg::Bool>(
		goal_published_topic_, 10, std::bind(&UartToMcuNode::goal_published_callback, this, std::placeholders::_1));

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

void UartToMcuNode::wheel_speeds_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
	if (msg->data.size() < 2) {
		RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "wheel_speeds message needs at least 2 values");
		return;
	}

	const auto clamp_to_i32_range = [](double value) {
		const double min_v = static_cast<double>(std::numeric_limits<int32_t>::min());
		const double max_v = static_cast<double>(std::numeric_limits<int32_t>::max());
		if (value < min_v) {
			return min_v;
		}
		if (value > max_v) {
			return max_v;
		}
		return value;
	};

	const double left_raw = clamp_to_i32_range(std::round(msg->data[0] * wheel_speed_scale_));
	const double right_raw = clamp_to_i32_range(std::round(msg->data[1] * wheel_speed_scale_));

	const int32_t left = static_cast<int32_t>(left_raw);
	const int32_t right = static_cast<int32_t>(right_raw);

	const auto packet = build_wheel_packet(left, right);
	if (!write_packet(packet)) {
		RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Failed writing packet to serial");
	}
}

void UartToMcuNode::goal_reached_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
	if (!msg->data) {
		return;
	}

	const auto packet = build_single_byte_packet(kGoalReachedPacketId, 0x00U);
	if (!write_packet(packet)) {
		RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Failed writing goal-reached packet to serial");
	}
}

void UartToMcuNode::goal_published_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
	if (!msg->data) {
		return;
	}

	const uint8_t packet_id = start_flag_ ? kGoalPublishedPacketId : kGoalPosePacketId;
	const auto packet = build_single_byte_packet(packet_id, 0x00U);
	if (!write_packet(packet)) {
		RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
			"Failed writing goal packet (id=0x%02X) to serial", packet_id);
		return;
	}

	if (start_flag_) {
		RCLCPP_INFO(get_logger(), "Sent start packet (id=0x%02X) on first goal", packet_id);
		start_flag_ = false;
	}
}

std::vector<uint8_t> UartToMcuNode::build_wheel_packet(int32_t left_value, int32_t right_value) const
{
	std::vector<uint8_t> packet;
	packet.reserve(2 + 1 + 4 + kWheelPayloadLength + 2);

	append_u16_le(packet, kFrameHeader);
	packet.push_back(kWheelPacketId);
	append_u32_le(packet, kWheelPayloadLength);
	append_i32_le(packet, left_value);
	append_i32_le(packet, right_value);
	append_u16_le(packet, kFrameTail);

	return packet;
}

std::vector<uint8_t> UartToMcuNode::build_single_byte_packet(uint8_t packet_id, uint8_t value) const
{
	std::vector<uint8_t> packet;
	packet.reserve(2 + 1 + 4 + 1 + 2);

	append_u16_le(packet, kFrameHeader);
	packet.push_back(packet_id);
	append_u32_le(packet, 1U);
	packet.push_back(value);
	append_u16_le(packet, kFrameTail);

	return packet;
}

bool UartToMcuNode::write_packet(const std::vector<uint8_t> & packet)
{
	if (serial_fd_ < 0 && !open_serial()) {
		return false;
	}

	size_t total_written = 0;
	while (total_written < packet.size()) {
		const ssize_t written = ::write(serial_fd_, packet.data() + total_written, packet.size() - total_written);
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

void UartToMcuNode::append_u32_le(std::vector<uint8_t> & out, uint32_t value)
{
	out.push_back(static_cast<uint8_t>(value & 0xFF));
	out.push_back(static_cast<uint8_t>((value >> 8) & 0xFF));
	out.push_back(static_cast<uint8_t>((value >> 16) & 0xFF));
	out.push_back(static_cast<uint8_t>((value >> 24) & 0xFF));
}

void UartToMcuNode::append_i32_le(std::vector<uint8_t> & out, int32_t value)
{
	append_u32_le(out, static_cast<uint32_t>(value));
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
