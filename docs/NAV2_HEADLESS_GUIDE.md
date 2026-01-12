# Nav2 Headless 运行指南

## 🎯 目标
在无图形界面环境下运行Nav2导航系统。

---

## ✅ 修改后的脚本

已创建 `script/nav2_headless.sh`，主要改动：

1. **移除了RViz启动**：最后一行不再启动 `rviz2`
2. **添加了监控提示**：显示如何通过命令行监控系统
3. **保持所有核心功能**：Nav2、定位、建图等功能完全保留

---

## 📋 使用方法

### 方法1：使用新的Headless脚本

```bash
# 给脚本执行权限
chmod +x script/nav2_headless.sh

# 运行
./script/nav2_headless.sh
```

### 方法2：修改原脚本（移除RViz）

在你的原脚本中，**删除或注释掉最后一行**：

```bash
# 注释掉这行：
# ros2 run rviz2 rviz2 -d $(ros2 pkg prefix nav2_bringup)/share/nav2_bringup/rviz/nav2_default_view.rviz

# 或者替换为：
echo ">>> Nav2 正在 Headless 模式下运行"
echo ">>> 使用命令行工具监控系统状态"
```

---

## 🔍 Headless模式下的监控方法

### 1. 查看节点状态

```bash
# 查看所有运行的节点
ros2 node list

# 查看特定节点的信息
ros2 node info /bt_navigator
ros2 node info /planner_server
```

### 2. 查看话题数据

```bash
# 查看机器人当前位置
ros2 topic echo /amcl_pose

# 查看激光扫描数据（只显示一次）
ros2 topic echo /scan --once

# 查看速度命令
ros2 topic echo /cmd_vel

# 查看所有话题
ros2 topic list
```

### 3. 查看话题频率

```bash
# 查看扫描频率
ros2 topic hz /scan

# 查看定位更新频率
ros2 topic hz /amcl_pose
```

### 4. 发送导航目标（命令行）

```bash
# 发送导航目标
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 1.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}"

# 或者使用更简单的格式
ros2 topic pub /goal_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'map'}, pose: {position: {x: 1.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}" --once
```

### 5. 查看TF树

```bash
# 查看TF树结构
ros2 run tf2_tools view_frames

# 这会生成 frames.pdf 文件，可以下载查看
```

---

## 🖥️ 如果需要可视化（远程）

### 方法1：X11转发（SSH）

```bash
# SSH登录时启用X11转发
ssh -X -p 7913 nyu@42.192.208.124

# 然后在远程运行RViz
ros2 run rviz2 rviz2
```

**注意**：需要本地有X服务器（Windows需要Xming或VcXsrv）

### 方法2：VNC（推荐）

```bash
# 在Jetson上安装VNC服务器
sudo apt-get install tigervnc-standalone-server tigervnc-common

# 启动VNC服务器
vncserver :1

# 在本地使用VNC客户端连接
# 地址：Jetson_IP:5901
```

### 方法3：使用Web界面

Nav2有一些Web界面工具，但需要额外配置。

---

## 📊 监控脚本示例

创建一个简单的监控脚本：

```bash
#!/bin/bash
# monitor_nav2.sh

echo "=== Nav2 系统监控 ==="
echo ""

echo "1. 节点状态："
ros2 node list
echo ""

echo "2. 话题列表："
ros2 topic list | head -20
echo ""

echo "3. 当前机器人位置："
ros2 topic echo /amcl_pose --once
echo ""

echo "4. 扫描数据（最后一条）："
ros2 topic echo /scan --once | head -5
echo ""

echo "5. 速度命令（最后一条）："
ros2 topic echo /cmd_vel --once
```

---

## 🔧 常见问题

### Q1: Nav2启动失败，提示需要显示

**解决方法**：
```bash
# 设置虚拟显示
export DISPLAY=:0

# 或者在启动脚本中添加
export DISPLAY=:0
ros2 launch nav2_bringup bringup_launch.py ...
```

### Q2: 如何确认Nav2正在运行？

**检查方法**：
```bash
# 查看节点
ros2 node list | grep nav

# 应该看到：
# /bt_navigator
# /planner_server
# /controller_server
# /recoveries_server
# /local_costmap/local_costmap
# /global_costmap/global_costmap
```

### Q3: 如何发送导航命令？

**方法1：使用action**
```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 1.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}"
```

**方法2：使用Python脚本**
```python
import rclpy
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped

rclpy.init()
node = rclpy.create_node('nav_client')
client = ActionClient(node, NavigateToPose, '/navigate_to_pose')

goal = NavigateToPose.Goal()
goal.pose.header.frame_id = 'map'
goal.pose.pose.position.x = 1.0
goal.pose.pose.position.y = 0.0
goal.pose.pose.orientation.w = 1.0

client.wait_for_server()
client.send_goal_async(goal)
```

---

## 📝 完整Headless工作流程

1. **启动系统**：
   ```bash
   ./script/nav2_headless.sh
   ```

2. **监控状态**（在另一个终端）：
   ```bash
   # 查看节点
   ros2 node list
   
   # 查看位置
   ros2 topic echo /amcl_pose
   ```

3. **发送导航目标**：
   ```bash
   ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
     "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 1.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}"
   ```

4. **监控导航进度**：
   ```bash
   ros2 topic echo /navigate_to_pose/_action/status
   ```

---

## 💡 提示

- **Nav2本身不需要GUI**：可以在headless模式下完全运行
- **RViz只是可视化工具**：用于调试和监控，不是必需的
- **命令行工具足够**：`ros2 topic echo`、`ros2 node list` 等可以完成大部分监控任务
- **远程可视化**：如果需要，使用VNC或X11转发

---

## 🎯 快速参考

```bash
# 启动（Headless）
./script/nav2_headless.sh

# 监控（另一个终端）
ros2 node list                    # 查看节点
ros2 topic echo /amcl_pose       # 查看位置
ros2 topic echo /cmd_vel         # 查看速度命令

# 发送目标
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 1.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}"
```
