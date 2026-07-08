#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace uart_to_mcu
{
inline constexpr const char * kDefaultWheelSpeedsTopic = "/wheel_speeds";
inline constexpr const char * kDefaultGoalReachedTopic = "/goal_reached";
inline constexpr const char * kDefaultGoalPublishedTopic = "/goal_published";
inline constexpr uint16_t kFrameHeader = 0xAA55U;
inline constexpr uint16_t kFrameTail = 0x55AAU;
inline constexpr uint8_t kWheelPacketId = 0x32U;
inline constexpr uint32_t kWheelPayloadLength = 8U;
inline constexpr uint8_t kGoalReachedPacketId = 0x35U;
inline constexpr uint8_t kGoalPublishedPacketId = 0x38U;
inline constexpr uint8_t kGoalPosePacketId = 0x36U;

class UartToMcuNode : public rclcpp::Node
{
public:
	UartToMcuNode();
	~UartToMcuNode() override;

private:
	void wheel_speeds_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
	void goal_reached_callback(const std_msgs::msg::Bool::SharedPtr msg);
	void goal_published_callback(const std_msgs::msg::Bool::SharedPtr msg);
	bool open_serial();
	void close_serial();
	std::vector<uint8_t> build_wheel_packet(int32_t left_value, int32_t right_value) const;
	std::vector<uint8_t> build_single_byte_packet(uint8_t packet_id, uint8_t value) const;
	bool write_packet(const std::vector<uint8_t> & packet);
	static void append_u16_le(std::vector<uint8_t> & out, uint16_t value);
	static void append_u32_le(std::vector<uint8_t> & out, uint32_t value);
	static void append_i32_le(std::vector<uint8_t> & out, int32_t value);

	std::string wheel_speeds_topic_;
	std::string goal_reached_topic_;
	std::string goal_published_topic_;
	std::string serial_port_;
	double wheel_speed_scale_;

	rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr wheel_sub_;
	rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr goal_reached_sub_;
	rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr goal_published_sub_;

	int serial_fd_;
	bool start_flag_;  // true 表示尚未发送启动包，下一次目标点触发开始数据包
};
}  // namespace uart_to_mcu
