#!/usr/bin/env python3
"""
IMU绝对角度读取脚本 - 从USB CDC串口读取并显示IMU姿态数据
使用方法: python3 read_imu.py [串口设备] [--save 文件名.csv] [--plot]
"""

import serial
import sys
import time
import argparse
from datetime import datetime

def find_serial_port():
    """自动查找USB CDC串口"""
    import serial.tools.list_ports

    ports = serial.tools.list_ports.comports()
    for port in ports:
        if 'usbmodem' in port.device or 'ttyACM' in port.device:
            print(f"找到USB CDC设备: {port.device} ({port.description})")
            return port.device

    print("未找到USB CDC设备，显示所有可用串口:")
    for port in ports:
        print(f"  {port.device}: {port.description}")

    return None

def parse_imu_line(line):
    """
    解析IMU行
    格式: IMU,timestamp,yaw,pitch,roll,yaw_total,round_count,gx,gy,gz
    """
    parts = line.strip().split(',')
    if len(parts) != 10 or parts[0] != 'IMU':
        return None

    try:
        data = {
            'timestamp': int(parts[1]),
            'yaw': float(parts[2]),
            'pitch': float(parts[3]),
            'roll': float(parts[4]),
            'yaw_total': float(parts[5]),
            'round_count': int(parts[6]),
            'gyro_x': float(parts[7]),
            'gyro_y': float(parts[8]),
            'gyro_z': float(parts[9])
        }
        return data
    except (ValueError, IndexError):
        return None

def main():
    parser = argparse.ArgumentParser(description='读取云台IMU姿态角度')
    parser.add_argument('port', nargs='?', default=None,
                        help='串口设备 (留空自动查找)')
    parser.add_argument('--save', metavar='FILE', default=None,
                        help='保存数据到CSV文件')
    parser.add_argument('--baud', type=int, default=115200,
                        help='波特率 (默认: 115200)')
    parser.add_argument('--plot', action='store_true',
                        help='实时绘图（需要matplotlib）')

    args = parser.parse_args()

    # 查找串口
    port = args.port
    if port is None:
        port = find_serial_port()
        if port is None:
            print("错误: 未找到USB CDC设备")
            sys.exit(1)

    print(f"正在连接到 {port} (波特率: {args.baud})...")

    try:
        ser = serial.Serial(port, args.baud, timeout=1)
        time.sleep(0.5)
        print(f"已连接！正在读取IMU数据... (按Ctrl+C退出)\n")
    except serial.SerialException as e:
        print(f"错误: 无法打开串口 {port}: {e}")
        sys.exit(1)

    # 打开CSV文件（如果需要）
    csv_file = None
    if args.save:
        csv_file = open(args.save, 'w')
        csv_file.write("system_time,timestamp_ms,yaw,pitch,roll,yaw_total_angle,round_count,gyro_x,gyro_y,gyro_z\n")
        print(f"数据将保存到: {args.save}\n")

    # 实时绘图（如果启用）
    plotter = None
    if args.plot:
        try:
            import matplotlib.pyplot as plt
            import matplotlib.animation as animation
            from collections import deque

            class RealtimePlotter:
                def __init__(self, max_points=200):
                    self.max_points = max_points
                    self.times = deque(maxlen=max_points)
                    self.yaw = deque(maxlen=max_points)
                    self.pitch = deque(maxlen=max_points)
                    self.roll = deque(maxlen=max_points)
                    self.yaw_total = deque(maxlen=max_points)

                    self.fig, (self.ax1, self.ax2) = plt.subplots(2, 1, figsize=(12, 8))
                    self.fig.suptitle('云台IMU实时姿态角度')

                    # 上图：单圈角度
                    self.line_yaw, = self.ax1.plot([], [], 'r-', label='Yaw', linewidth=2)
                    self.line_pitch, = self.ax1.plot([], [], 'g-', label='Pitch', linewidth=2)
                    self.line_roll, = self.ax1.plot([], [], 'b-', label='Roll', linewidth=2)
                    self.ax1.set_xlabel('时间 (s)')
                    self.ax1.set_ylabel('角度 (°)')
                    self.ax1.set_title('单圈角度 (-180° ~ +180°)')
                    self.ax1.legend(loc='upper right')
                    self.ax1.grid(True, alpha=0.3)
                    self.ax1.set_ylim(-200, 200)

                    # 下图：多圈累积角度
                    self.line_yaw_total, = self.ax2.plot([], [], 'r-', label='YawTotalAngle', linewidth=2)
                    self.ax2.set_xlabel('时间 (s)')
                    self.ax2.set_ylabel('累积角度 (°)')
                    self.ax2.set_title('Yaw多圈累积角度（支持小陀螺）')
                    self.ax2.legend(loc='upper right')
                    self.ax2.grid(True, alpha=0.3)

                    plt.tight_layout()

                def update(self, data):
                    timestamp_sec = data['timestamp'] / 1000.0
                    self.times.append(timestamp_sec)
                    self.yaw.append(data['yaw'])
                    self.pitch.append(data['pitch'])
                    self.roll.append(data['roll'])
                    self.yaw_total.append(data['yaw_total'])

                    if len(self.times) > 1:
                        # 更新上图
                        self.line_yaw.set_data(self.times, self.yaw)
                        self.line_pitch.set_data(self.times, self.pitch)
                        self.line_roll.set_data(self.times, self.roll)
                        self.ax1.set_xlim(self.times[0], self.times[-1])

                        # 更新下图
                        self.line_yaw_total.set_data(self.times, self.yaw_total)
                        self.ax2.set_xlim(self.times[0], self.times[-1])
                        yaw_min = min(self.yaw_total)
                        yaw_max = max(self.yaw_total)
                        yaw_range = yaw_max - yaw_min
                        self.ax2.set_ylim(yaw_min - yaw_range*0.1, yaw_max + yaw_range*0.1)

                        plt.pause(0.001)

                def show(self):
                    plt.show(block=False)

            plotter = RealtimePlotter()
            plotter.show()
            print("实时绘图已启用\n")

        except ImportError:
            print("警告: matplotlib未安装，无法启用实时绘图")
            print("安装方法: pip3 install matplotlib\n")
            args.plot = False

    # 打印表头
    print(f"\n{'='*95}")
    print(f"{'时间':<12} {'Yaw':>8} {'Pitch':>8} {'Roll':>8} | "
          f"{'Yaw累积':>10} {'圈数':>5} | {'陀螺仪 (rad/s)':<25}")
    print("-" * 95)

    try:
        line_buffer = ""
        while True:
            # 读取数据
            if ser.in_waiting > 0:
                chunk = ser.read(ser.in_waiting).decode('utf-8', errors='ignore')
                line_buffer += chunk

                # 按行处理
                while '\n' in line_buffer:
                    line, line_buffer = line_buffer.split('\n', 1)
                    line = line.strip()

                    if not line:  # 跳过空行
                        continue

                    # 处理IMU数据行
                    if line.startswith('IMU'):
                        data = parse_imu_line(line)
                        if data:
                            # 格式化输出
                            timestamp_sec = data['timestamp'] / 1000.0
                            print(f"{timestamp_sec:>10.2f}s "
                                  f"{data['yaw']:>8.2f} {data['pitch']:>8.2f} {data['roll']:>8.2f} | "
                                  f"{data['yaw_total']:>10.2f} {data['round_count']:>5d} | "
                                  f"({data['gyro_x']:>6.3f}, {data['gyro_y']:>6.3f}, {data['gyro_z']:>6.3f})")

                            # 保存到CSV
                            if csv_file:
                                system_time = datetime.now().isoformat()
                                csv_file.write(f"{system_time},"
                                               f"{data['timestamp']},"
                                               f"{data['yaw']:.2f},"
                                               f"{data['pitch']:.2f},"
                                               f"{data['roll']:.2f},"
                                               f"{data['yaw_total']:.2f},"
                                               f"{data['round_count']},"
                                               f"{data['gyro_x']:.6f},"
                                               f"{data['gyro_y']:.6f},"
                                               f"{data['gyro_z']:.6f}\n")
                                csv_file.flush()

                            # 更新绘图
                            if plotter:
                                plotter.update(data)
                    else:
                        # 只显示IMU相关的debug信息（过滤掉ENCODER, YAW_CSV, PITCH_CSV等）
                        imu_keywords = ['IMU', 'BMI088', 'Calibration', 'calibration', 'Gyro', 'Accel',
                                       'Init', 'offset', 'gNorm', 'Temp when cali']
                        if any(keyword in line for keyword in imu_keywords):
                            print(f"[DEBUG] {line}")

            else:
                time.sleep(0.01)

    except KeyboardInterrupt:
        print("\n\n接收到Ctrl+C，正在退出...")

    finally:
        if csv_file:
            csv_file.close()
            print(f"数据已保存到: {args.save}")
        ser.close()
        print("串口已关闭")

        if args.plot:
            print("关闭绘图窗口以退出...")
            try:
                import matplotlib.pyplot as plt
                plt.show()  # 阻塞直到窗口关闭
            except:
                pass

if __name__ == '__main__':
    main()
