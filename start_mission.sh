#!/usr/bin/env bash
#
# 一键拉起小车任务链路(在 RDK X5 上运行):
#   1) 雷达驱动 bluesea2
#   2) cartographer 建图/定位
#   3) car_mission 任务控制(PID 循迹 + 串口下发)
#
# 按顺序启动并在各阶段之间等待,确保 /scan 与 map->base_link 的 TF 先就绪,
# 再拉起依赖它们的 car_mission。Ctrl+C 会一键关闭全部节点。
#
# 常用覆盖(环境变量):
#   ROS_DISTRO=humble ./start_mission.sh      # 指定 ROS 2 版本
#   WS_DIR=/home/pi/ws ./start_mission.sh      # 指定工作空间(含 install/)
#   CARTO_WAIT=8 ./start_mission.sh            # 加长 cartographer 就绪等待
#
set -uo pipefail

# ---- 可配置项 ----
ROS_DISTRO="${ROS_DISTRO:-humble}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# 脚本位于 <workspace>/src/ 下,工作空间默认取其上一级(install/ 所在处)。
WS_DIR="${WS_DIR:-$(cd "$SCRIPT_DIR/.." && pwd)}"

LIDAR_CMD="${LIDAR_CMD:-ros2 launch bluesea2 uart_lidar.launch}"
CARTO_CMD="${CARTO_CMD:-ros2 launch cartographer_ros cartographer_live.launch.py}"
MISSION_CMD="${MISSION_CMD:-ros2 launch car_bringup car_mission.launch.py}"

# 各阶段启动后的等待秒数(给 topic / TF 建立时间)。
LIDAR_WAIT="${LIDAR_WAIT:-3}"
CARTO_WAIT="${CARTO_WAIT:-5}"

LOG_DIR="${LOG_DIR:-$SCRIPT_DIR/logs}"

# ---- 运行状态 ----
PIDS=()
NAMES=()
CLEANED=0

cleanup() {
	# 防止 INT/TERM 与 EXIT 陷阱重复执行。
	[[ "$CLEANED" -eq 1 ]] && return
	CLEANED=1
	echo
	echo "[stop] 正在关闭所有节点..."
	# 逆序关闭:先停任务,再停建图,最后停雷达。
	for ((i=${#PIDS[@]}-1; i>=0; i--)); do
		# setsid 使每个 launch 独占进程组,负号向整个进程组发信号,连带 ros2 launch 的子节点一起收。
		kill -INT -"${PIDS[$i]}" 2>/dev/null || true
	done
	wait 2>/dev/null || true
	echo "[stop] 已全部退出"
}
trap cleanup INT TERM EXIT

# ---- 环境 ----
if [[ ! -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
	echo "[error] 未找到 /opt/ros/${ROS_DISTRO}/setup.bash,请用 ROS_DISTRO=<版本> 指定" >&2
	exit 1
fi
# shellcheck disable=SC1090
source "/opt/ros/${ROS_DISTRO}/setup.bash"

if [[ -f "$WS_DIR/install/setup.bash" ]]; then
	# shellcheck disable=SC1090
	source "$WS_DIR/install/setup.bash"
else
	echo "[warn] 未找到 $WS_DIR/install/setup.bash;car_mission 可能无法启动。" >&2
	echo "       请先在工作空间执行 colcon build,或用 WS_DIR=<路径> 指定。" >&2
fi

mkdir -p "$LOG_DIR"

# start <名称> <命令...>:后台启动为独立进程组,输出写入日志。
start() {
	local name="$1"; shift
	echo "[start] ${name}: $*"
	setsid bash -c "$*" >"$LOG_DIR/${name}.log" 2>&1 &
	PIDS+=("$!")
	NAMES+=("$name")
}

echo "[info] ROS_DISTRO=${ROS_DISTRO}  WS_DIR=${WS_DIR}"
echo "[info] 日志目录: ${LOG_DIR}"
echo

start lidar        "$LIDAR_CMD"
sleep "$LIDAR_WAIT"

start cartographer "$CARTO_CMD"
sleep "$CARTO_WAIT"

start car_mission  "$MISSION_CMD"

echo
echo "[ok] 三个模块已拉起。发送任务指令示例:"
echo "     ros2 topic pub -1 /car_mission_start std_msgs/msg/String \"data: '1'\"   # 前往 A(2->B 3->C 0->停车)"
echo "[ok] Ctrl+C 一键停止全部。"
echo

# 任一模块退出即结束脚本(触发 cleanup 收拾其余进程)。
wait -n 2>/dev/null || wait
