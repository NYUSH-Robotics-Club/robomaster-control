#!/bin/bash
#
# setup_jetson.sh
# 
# 快速设置脚本，用于在Jetson上配置radar forwarder环境
#

set -e  # 遇到错误立即退出

echo "=========================================="
echo "Jetson Radar Forwarder 环境设置"
echo "=========================================="
echo ""

# 检查是否为root用户
if [ "$EUID" -eq 0 ]; then 
   echo "请不要使用root用户运行此脚本"
   exit 1
fi

# 步骤1：安装依赖
echo "[1/5] 安装Python依赖..."
sudo apt-get update
sudo apt-get install -y python3 python3-pip python3-serial screen

# 检查pyserial是否已安装
if ! python3 -c "import serial" 2>/dev/null; then
    echo "  安装pyserial..."
    pip3 install pyserial
else
    echo "  pyserial已安装"
fi

# 步骤2：设置用户组权限
echo ""
echo "[2/5] 设置串口权限..."
if groups | grep -q dialout; then
    echo "  用户已在dialout组中"
else
    echo "  将用户添加到dialout组..."
    sudo usermod -a -G dialout $USER
    echo "  ✓ 已添加到dialout组（需要重新登录才能生效）"
    echo "  ⚠️  或者运行: newgrp dialout"
fi

# 步骤3：查找STM32设备
echo ""
echo "[3/5] 查找STM32设备..."
STM32_PORT=""
if [ -e /dev/ttyACM0 ]; then
    STM32_PORT="/dev/ttyACM0"
elif [ -e /dev/ttyACM1 ]; then
    STM32_PORT="/dev/ttyACM1"
elif [ -e /dev/ttyUSB0 ]; then
    STM32_PORT="/dev/ttyUSB0"
else
    echo "  ⚠️  未找到STM32设备，请检查USB连接"
    echo "  尝试运行: ls -l /dev/ttyACM* /dev/ttyUSB*"
fi

if [ -n "$STM32_PORT" ]; then
    echo "  找到设备: $STM32_PORT"
    echo "  设置权限..."
    sudo chmod 666 $STM32_PORT
    echo "  ✓ 权限已设置"
fi

# 步骤4：创建udev规则（可选）
echo ""
echo "[4/5] 配置udev规则（可选）..."
UDEV_RULE="/etc/udev/rules.d/99-stm32-cdc.rules"
if [ -f "$UDEV_RULE" ]; then
    echo "  udev规则已存在"
else
    echo "  创建udev规则..."
    sudo tee $UDEV_RULE > /dev/null <<EOF
# STM32 Virtual COM Port
SUBSYSTEM=="tty", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", MODE="0666", GROUP="dialout"
EOF
    sudo udevadm control --reload-rules
    sudo udevadm trigger
    echo "  ✓ udev规则已创建"
fi

# 步骤5：测试连接
echo ""
echo "[5/5] 测试连接..."
if [ -n "$STM32_PORT" ]; then
    echo "  测试端口: $STM32_PORT"
    if python3 -c "import serial; s=serial.Serial('$STM32_PORT', 115200, timeout=0.1); s.close(); print('OK')" 2>/dev/null; then
        echo "  ✓ 端口可以打开"
    else
        echo "  ⚠️  端口无法打开，可能需要重新登录或检查权限"
    fi
else
    echo "  ⚠️  跳过测试（未找到设备）"
fi

# 完成
echo ""
echo "=========================================="
echo "设置完成！"
echo "=========================================="
echo ""
echo "下一步："
echo "1. 如果用户组已更改，请重新登录或运行: newgrp dialout"
echo "2. 查找STM32端口: ls -l /dev/ttyACM*"
echo "3. 测试发送命令:"
if [ -n "$STM32_PORT" ]; then
    echo "   python3 script/cmd_vel_forwarder.py --port $STM32_PORT --vx 0.5 --rate 20"
else
    echo "   python3 script/cmd_vel_forwarder.py --port /dev/ttyACM0 --vx 0.5 --rate 20"
fi
echo "4. ROS2模式:"
if [ -n "$STM32_PORT" ]; then
    echo "   python3 script/cmd_vel_forwarder.py --port $STM32_PORT --ros2"
else
    echo "   python3 script/cmd_vel_forwarder.py --port /dev/ttyACM0 --ros2"
fi
echo ""
