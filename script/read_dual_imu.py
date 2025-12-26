#!/usr/bin/env python3
"""
双IMU数据读取脚本 - 从USB CDC串口读取并显示BMI088和WT61C的IMU数据
支持实时显示、数据保存和绘图功能

使用方法:
    python3 read_dual_imu.py [串口设备] [选项]

选项:
    --save FILE     保存数据到CSV文件
    --plot          启用实时绘图
    --baud RATE     设置波特率（默认115200）
    --gimbal-only   只显示云台IMU
    --chassis-only  只显示底盘IMU
"""

import serial
import sys
import time
import argparse
from datetime import datetime
from collections import deque

# 颜色输出支持（兼容Windows和Unix）
class Colors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKCYAN = '\033[96m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'

    @staticmethod
    def disable():
        Colors.HEADER = ''
        Colors.OKBLUE = ''
        Colors.OKCYAN = ''
        Colors.OKGREEN = ''
        Colors.WARNING = ''
        Colors.FAIL = ''
        Colors.ENDC = ''
        Colors.BOLD = ''
        Colors.UNDERLINE = ''

# Windows系统禁用颜色
if sys.platform == 'win32':
    try:
        import colorama
        colorama.init()
    except ImportError:
        Colors.disable()

def find_serial_port():
    """自动查找USB CDC串口"""
    import serial.tools.list_ports

    ports = serial.tools.list_ports.comports()
    for port in ports:
        if 'usbmodem' in port.device or 'ttyACM' in port.device:
            print(f"找到USB CDC设备: {port.device} ({port.description})")
            return port.device

    print(f"未找到USB CDC设备，显示所有可用串口:")
    for port in ports:
        print(f"  {port.device}: {port.description}")

    return None

def parse_imu_line(line):
    """
    解析IMU数据行
    格式: IMU,timestamp,gimbal(8字段),chassis(9字段)

    Gimbal (BMI088): yaw, pitch, roll, yaw_total, round_count, gx, gy, gz
    Chassis (WT61C): yaw, pitch, roll, gx, gy, gz, ax, ay, az
    """
    parts = line.strip().split(',')
    if parts[0] != 'IMU':
        return None

    if len(parts) != 19:
        # 调试：打印字段数不匹配的行
        print(f"[ERROR] 字段数不匹配: 期望19个，实际{len(parts)}个")
        print(f"[ERROR] 数据: {line[:100]}...")
        return None

    try:
        data = {
            'timestamp': int(parts[1]),
            'gimbal': {
                'yaw': float(parts[2]),
                'pitch': float(parts[3]),
                'roll': float(parts[4]),
                'yaw_total': float(parts[5]),
                'round_count': int(parts[6]),
                'gyro_x': float(parts[7]),
                'gyro_y': float(parts[8]),
                'gyro_z': float(parts[9])
            },
            'chassis': {
                'yaw': float(parts[10]),
                'pitch': float(parts[11]),
                'roll': float(parts[12]),
                'gyro_x': float(parts[13]),
                'gyro_y': float(parts[14]),
                'gyro_z': float(parts[15]),
                'accel_x': float(parts[16]),
                'accel_y': float(parts[17]),
                'accel_z': float(parts[18])
            }
        }
        return data
    except (ValueError, IndexError) as e:
        return None

class DualIMUDisplay:
    """双IMU数据显示类"""

    def __init__(self, show_gimbal=True, show_chassis=True):
        self.show_gimbal = show_gimbal
        self.show_chassis = show_chassis
        self.frame_count = 0

    def print_header(self):
        """打印表头"""
        print(f"\n{'='*120}")

        if self.show_gimbal and self.show_chassis:
            print(f"{'时间':<12} | {'云台IMU (BMI088)':<60} | {'底盘IMU (WT61C)':<40}")
            print(f"{' '*12} | {'Yaw':>8} {'Pitch':>8} {'Roll':>8} {'Total':>9} {'Rnd':>3} {'Gyro(rad/s)':<19} | {'Yaw':>8} {'Pitch':>8} {'Roll':>8} {'Gyro(rad/s)':<15}")
        elif self.show_gimbal:
            print(f"{'时间':<12} | {'云台IMU (BMI088)':<60}")
            print(f"{' '*12} | {'Yaw':>8} {'Pitch':>8} {'Roll':>8} {'YawTotal':>10} {'Rounds':>5} {'Gyro (rad/s)':<22}")
        else:
            print(f"{'时间':<12} | {'底盘IMU (WT61C)':<40}")
            print(f"{' '*12} | {'Yaw':>8} {'Pitch':>8} {'Roll':>8} {'Gyro (rad/s)':<22}")

        print(f"{'-'*120}")

    def print_data(self, data):
        """打印IMU数据"""
        timestamp_sec = data['timestamp'] / 1000.0

        if self.show_gimbal and self.show_chassis:
            g = data['gimbal']
            c = data['chassis']
            print(f"{timestamp_sec:>10.2f}s | "
                  f"{g['yaw']:>8.2f} {g['pitch']:>8.2f} {g['roll']:>8.2f} "
                  f"{g['yaw_total']:>9.2f} {g['round_count']:>3d} "
                  f"({g['gyro_x']:>5.3f},{g['gyro_y']:>5.3f},{g['gyro_z']:>5.3f}) | "
                  f"{c['yaw']:>8.2f} {c['pitch']:>8.2f} {c['roll']:>8.2f} "
                  f"({c['gyro_x']:>5.3f},{c['gyro_y']:>5.3f},{c['gyro_z']:>5.3f})")
        elif self.show_gimbal:
            g = data['gimbal']
            print(f"{timestamp_sec:>10.2f}s | "
                  f"{g['yaw']:>8.2f} {g['pitch']:>8.2f} {g['roll']:>8.2f} "
                  f"{g['yaw_total']:>10.2f} {g['round_count']:>5d} "
                  f"({g['gyro_x']:>6.3f},{g['gyro_y']:>6.3f},{g['gyro_z']:>6.3f})")
        else:
            c = data['chassis']
            print(f"{timestamp_sec:>10.2f}s | "
                  f"{c['yaw']:>8.2f} {c['pitch']:>8.2f} {c['roll']:>8.2f} "
                  f"({c['gyro_x']:>6.3f},{c['gyro_y']:>6.3f},{c['gyro_z']:>6.3f})")

        # 每20行重新打印表头
        self.frame_count += 1
        if self.frame_count % 20 == 0:
            self.print_header()

class RealtimePlotter:
    """实时数据绘图类"""

    def __init__(self, max_points=200, show_gimbal=True, show_chassis=True):
        import matplotlib.pyplot as plt

        self.max_points = max_points
        self.show_gimbal = show_gimbal
        self.show_chassis = show_chassis

        # 数据缓冲
        self.times = deque(maxlen=max_points)

        if show_gimbal:
            self.g_yaw = deque(maxlen=max_points)
            self.g_pitch = deque(maxlen=max_points)
            self.g_roll = deque(maxlen=max_points)
            self.g_yaw_total = deque(maxlen=max_points)

        if show_chassis:
            self.c_yaw = deque(maxlen=max_points)
            self.c_pitch = deque(maxlen=max_points)
            self.c_roll = deque(maxlen=max_points)

        # 创建图形
        num_plots = 0
        if show_gimbal:
            num_plots += 2
        if show_chassis:
            num_plots += 1

        self.fig, self.axes = plt.subplots(num_plots, 1, figsize=(12, 4*num_plots))
        if num_plots == 1:
            self.axes = [self.axes]

        self.fig.suptitle('双IMU实时数据监控', fontsize=14, fontweight='bold')

        ax_idx = 0

        # 云台IMU单圈角度
        if show_gimbal:
            ax = self.axes[ax_idx]
            self.line_g_yaw, = ax.plot([], [], 'r-', label='Yaw', linewidth=2)
            self.line_g_pitch, = ax.plot([], [], 'g-', label='Pitch', linewidth=2)
            self.line_g_roll, = ax.plot([], [], 'b-', label='Roll', linewidth=2)
            ax.set_xlabel('时间 (s)')
            ax.set_ylabel('角度 (°)')
            ax.set_title('云台IMU (BMI088) - 单圈角度', fontweight='bold')
            ax.legend(loc='upper right')
            ax.grid(True, alpha=0.3)
            ax.set_ylim(-200, 200)
            ax_idx += 1

            # 云台IMU多圈累积
            ax = self.axes[ax_idx]
            self.line_g_yaw_total, = ax.plot([], [], 'r-', label='YawTotalAngle', linewidth=2)
            ax.set_xlabel('时间 (s)')
            ax.set_ylabel('累积角度 (°)')
            ax.set_title('云台IMU (BMI088) - Yaw多圈累积', fontweight='bold')
            ax.legend(loc='upper right')
            ax.grid(True, alpha=0.3)
            ax_idx += 1

        # 底盘IMU姿态角
        if show_chassis:
            ax = self.axes[ax_idx]
            self.line_c_yaw, = ax.plot([], [], 'r-', label='Yaw', linewidth=2)
            self.line_c_pitch, = ax.plot([], [], 'g-', label='Pitch', linewidth=2)
            self.line_c_roll, = ax.plot([], [], 'b-', label='Roll', linewidth=2)
            ax.set_xlabel('时间 (s)')
            ax.set_ylabel('角度 (°)')
            ax.set_title('底盘IMU (WT61C) - 姿态角度', fontweight='bold')
            ax.legend(loc='upper right')
            ax.grid(True, alpha=0.3)
            ax.set_ylim(-200, 200)
            ax_idx += 1

        plt.tight_layout()

    def update(self, data):
        """更新绘图数据"""
        import matplotlib.pyplot as plt

        timestamp_sec = data['timestamp'] / 1000.0
        self.times.append(timestamp_sec)

        ax_idx = 0

        if self.show_gimbal:
            g = data['gimbal']
            self.g_yaw.append(g['yaw'])
            self.g_pitch.append(g['pitch'])
            self.g_roll.append(g['roll'])
            self.g_yaw_total.append(g['yaw_total'])

            if len(self.times) > 1:
                # 更新单圈角度
                ax = self.axes[ax_idx]
                self.line_g_yaw.set_data(self.times, self.g_yaw)
                self.line_g_pitch.set_data(self.times, self.g_pitch)
                self.line_g_roll.set_data(self.times, self.g_roll)
                ax.set_xlim(self.times[0], self.times[-1])
                ax_idx += 1

                # 更新多圈累积
                ax = self.axes[ax_idx]
                self.line_g_yaw_total.set_data(self.times, self.g_yaw_total)
                ax.set_xlim(self.times[0], self.times[-1])
                yaw_min = min(self.g_yaw_total)
                yaw_max = max(self.g_yaw_total)
                yaw_range = max(yaw_max - yaw_min, 10)
                ax.set_ylim(yaw_min - yaw_range*0.1, yaw_max + yaw_range*0.1)
                ax_idx += 1

        if self.show_chassis:
            c = data['chassis']
            self.c_yaw.append(c['yaw'])
            self.c_pitch.append(c['pitch'])
            self.c_roll.append(c['roll'])

            if len(self.times) > 1:
                ax = self.axes[ax_idx]
                self.line_c_yaw.set_data(self.times, self.c_yaw)
                self.line_c_pitch.set_data(self.times, self.c_pitch)
                self.line_c_roll.set_data(self.times, self.c_roll)
                ax.set_xlim(self.times[0], self.times[-1])
                ax_idx += 1

        plt.pause(0.001)

    def show(self):
        """显示绘图窗口"""
        import matplotlib.pyplot as plt
        plt.show(block=False)

def main():
    parser = argparse.ArgumentParser(
        description='读取云台(BMI088)和底盘(WT61C)双IMU数据',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  %(prog)s                          # 自动查找串口，显示两个IMU
  %(prog)s /dev/ttyACM0             # 指定串口
  %(prog)s --plot                   # 启用实时绘图
  %(prog)s --save imu_data.csv      # 保存数据到文件
  %(prog)s --gimbal-only            # 只显示云台IMU
  %(prog)s --chassis-only           # 只显示底盘IMU
        """)

    parser.add_argument('port', nargs='?', default=None,
                        help='串口设备 (留空自动查找)')
    parser.add_argument('--save', metavar='FILE', default=None,
                        help='保存数据到CSV文件')
    parser.add_argument('--baud', type=int, default=115200,
                        help='波特率 (默认: 115200)')
    parser.add_argument('--plot', action='store_true',
                        help='启用实时绘图（需要matplotlib）')
    parser.add_argument('--gimbal-only', action='store_true',
                        help='只显示云台IMU (BMI088)')
    parser.add_argument('--chassis-only', action='store_true',
                        help='只显示底盘IMU (WT61C)')

    args = parser.parse_args()

    # 确定显示哪些IMU
    show_gimbal = not args.chassis_only
    show_chassis = not args.gimbal_only

    # 查找串口
    port = args.port
    if port is None:
        port = find_serial_port()
        if port is None:
            print(f"错误: 未找到USB CDC设备")
            sys.exit(1)

    print(f"正在连接到 {port} (波特率: {args.baud})...")

    try:
        ser = serial.Serial(port, args.baud, timeout=1)
        time.sleep(0.5)
        print(f"已连接！正在读取IMU数据... (按Ctrl+C退出)\n")
    except serial.SerialException as e:
        print(f"错误: 无法打开串口 {port}: {e}")
        sys.exit(1)

    # 打开CSV文件
    csv_file = None
    if args.save:
        csv_file = open(args.save, 'w')
        csv_file.write("system_time,timestamp_ms,")
        csv_file.write("g_yaw,g_pitch,g_roll,g_yaw_total,g_round_count,g_gyro_x,g_gyro_y,g_gyro_z,")
        csv_file.write("c_yaw,c_pitch,c_roll,c_gyro_x,c_gyro_y,c_gyro_z,c_accel_x,c_accel_y,c_accel_z\n")
        print(f"数据将保存到: {args.save}\n")

    # 实时绘图
    plotter = None
    if args.plot:
        try:
            plotter = RealtimePlotter(max_points=200,
                                     show_gimbal=show_gimbal,
                                     show_chassis=show_chassis)
            plotter.show()
            print(f"实时绘图已启用\n")
        except ImportError:
            print(f"警告: matplotlib未安装，无法启用实时绘图")
            print("安装方法: pip3 install matplotlib\n")
            args.plot = False

    # 创建显示对象
    display = DualIMUDisplay(show_gimbal=show_gimbal, show_chassis=show_chassis)
    display.print_header()

    try:
        line_buffer = ""
        data_count = 0

        while True:
            # 读取数据
            if ser.in_waiting > 0:
                chunk = ser.read(ser.in_waiting).decode('utf-8', errors='ignore')
                line_buffer += chunk

                # 按行处理
                while '\n' in line_buffer:
                    line, line_buffer = line_buffer.split('\n', 1)
                    line = line.strip()

                    if not line:
                        continue

                    # 处理IMU数据行
                    if line.startswith('IMU'):
                        data = parse_imu_line(line)
                        if data:
                            data_count += 1

                            # 显示数据
                            display.print_data(data)

                            # 保存到CSV
                            if csv_file:
                                system_time = datetime.now().isoformat()
                                g = data['gimbal']
                                c = data['chassis']
                                csv_file.write(f"{system_time},{data['timestamp']},"
                                             f"{g['yaw']:.2f},{g['pitch']:.2f},{g['roll']:.2f},"
                                             f"{g['yaw_total']:.2f},{g['round_count']},"
                                             f"{g['gyro_x']:.6f},{g['gyro_y']:.6f},{g['gyro_z']:.6f},"
                                             f"{c['yaw']:.2f},{c['pitch']:.2f},{c['roll']:.2f},"
                                             f"{c['gyro_x']:.6f},{c['gyro_y']:.6f},{c['gyro_z']:.6f},"
                                             f"{c['accel_x']},{c['accel_y']},{c['accel_z']}\n")
                                csv_file.flush()

                            # 更新绘图
                            if plotter:
                                plotter.update(data)
                    else:
                        # 只显示IMU相关的debug信息
                        imu_keywords = ['IMU', 'BMI088', 'WT61C', 'Calibration', 'calibration',
                                       'Gyro', 'Accel', 'Init', 'offset', 'gNorm', 'Temp when cali',
                                       'gyro_data_update', 'About to print']
                        if any(keyword in line for keyword in imu_keywords):
                            print(f"[DEBUG] {line}")

            else:
                time.sleep(0.01)

    except KeyboardInterrupt:
        print(f"\n\n接收到Ctrl+C，正在退出...")
        print(f"共接收 {data_count} 条IMU数据")

    finally:
        if csv_file:
            csv_file.close()
            print(f"数据已保存到: {args.save}")
        ser.close()
        print(f"串口已关闭")

        if args.plot:
            print("关闭绘图窗口以退出...")
            try:
                import matplotlib.pyplot as plt
                plt.show()
            except:
                pass

if __name__ == '__main__':
    main()
