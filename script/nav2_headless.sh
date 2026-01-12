#!/bin/bash

# Nav2 Headless 启动脚本
# 适用于无图形界面的环境（SSH、服务器等）

# Ctrl+C 退出时自动清理所有后台进程
trap 'echo "正在关闭所有节点..."; kill $(jobs -p); exit' SIGINT

# ==========================================
# [新增步骤] 强力清理旧环境 (放在最最前面)
# ==========================================
echo ">>> [清理] 正在强制杀掉旧进程并清理 DDS 共享内存..."

# 1. 强制杀掉相关节点的进程 (2>/dev/null 表示不显示"未找到进程"的报错)
killall -9 component_container_isolated unitree_lidar_ros2_node point_lio_node pointlio_mapping python3 2>/dev/null

# 2. 停止 ROS2 守护进程 (往往是造成节点无法发现的元凶)
ros2 daemon stop

# 3. 清理 FastDDS 留下的共享内存文件 (解决 shm 报错)
sudo rm -rf /dev/shm/fastrtps*

echo ">>> 环境清理完毕，等待 1 秒..."
sleep 1

# ==========================================
# [步骤 0] 硬件初始化
# ==========================================
echo ">>> [0/8] 正在搜索并执行雷达串口初始化工具..."
find ~/nav_ws -name "example_lidar_serial" -type f -exec sudo {} \;

# 给硬件一点反应时间，防止端口还没准备好驱动就启动了
echo ">>> 等待串口重置生效 (2秒)..."
sleep 2

# ==========================================
# 原有流程开始
# ==========================================

echo ">>> [1/8] 初始化 ROS 环境..."
source /opt/ros/humble/setup.bash
cd ~/nav_ws
source install/setup.bash

echo ">>> [2/8] 再次确认串口权限 (双重保险)..."
if [ -e /dev/ttyACM0 ]; then
    sudo chmod 777 /dev/ttyACM0
else
    echo "警告：未找到 /dev/ttyACM0，可能是上一步初始化更改了端口名，或者雷达未连接。"
fi

echo ">>> [3/8] 启动 Unitree 激光雷达驱动..."
ros2 launch unitree_lidar_ros2 launch.py &
PID_LIDAR=$!
sleep 2

echo ">>> [4/8] 启动 Point-LIO (建图/定位核心)..."
ros2 launch point_lio mapping_unilidar_l2.launch.py &

echo ">>> 等待 Point-LIO 初始化 (5秒)..."
sleep 5

echo ">>> [5/8] 发布静态 TF 变换..."
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 odom camera_init &
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 aft_mapped base_link &
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_link unilidar_imu_initial &
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_link base_footprint &

echo ">>> [6/8] 启动 Pointcloud 转 LaserScan (极速模式)..."
ros2 run pointcloud_to_laserscan pointcloud_to_laserscan_node --ros-args \
-p target_frame:=base_link \
-p transform_tolerance:=0.2 \
-p min_height:=0.05 \
-p max_height:=0.5 \
-p angle_min:=-3.1415 \
-p angle_max:=3.1415 \
-p angle_increment:=0.08 \
-p scan_time:=0.1 \
-p range_min:=0.2 \
-p range_max:=6.0 \
-p use_inf:=true \
-p inf_epsilon:=1.0 \
-p queue_size:=1 \
-r cloud_in:=/unilidar/cloud \
-r scan:=/scan \
--ros-args -p qos_overrides./parameter_events.publisher.reliability:=best_effort &

echo ">>> [7/8] 启动 Nav2 导航 (Headless模式)..."
# 设置环境变量禁用GUI相关功能
export DISPLAY=:0  # 即使没有显示器也设置一个虚拟显示
ros2 launch nav2_bringup bringup_launch.py \
    use_sim_time:=False \
    map:=/home/nyu/Desktop/map/1_map.yaml \
    params_file:=/home/nyu/nav_ws/my_nav2_params.yaml &

echo ">>> 等待 Nav2 启动 (10秒)..."
sleep 10

echo ">>> [8/8] 自动设置初始位置 (发送 5 次)..."
for i in {1..5}
do
   ros2 topic pub -1 /initialpose geometry_msgs/msg/PoseWithCovarianceStamped "{header: {frame_id: 'map'}, pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}" > /dev/null
   sleep 1
done

echo ">>> --------------------------------------"
echo ">>> 系统全部就绪！Nav2 正在 Headless 模式下运行。"
echo ">>> --------------------------------------"
echo ""
echo ">>> 监控命令："
echo ">>>   查看节点状态: ros2 node list"
echo ">>>   查看话题: ros2 topic list"
echo ">>>   查看机器人位置: ros2 topic echo /amcl_pose"
echo ">>>   查看导航目标: ros2 topic echo /navigate_to_pose/_action/status"
echo ">>>   发送导航目标: ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \"{pose: {header: {frame_id: 'map'}, pose: {position: {x: 1.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}\""
echo ""

# ==========================================
# Headless 模式：不启动 RViz
# ==========================================
# 如果需要监控，可以使用以下命令：
# - ros2 topic echo /amcl_pose
# - ros2 topic echo /scan
# - ros2 topic echo /cmd_vel
# - ros2 node list
# - ros2 topic list

# 保持脚本运行，等待 Ctrl+C
echo ">>> 按 Ctrl+C 退出..."
wait
