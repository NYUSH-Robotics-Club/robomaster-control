#!/usr/bin/env python3
"""
编码器值读取脚本 - 从USB CDC串口读取并过滤ENCODER数据
使用方法: python3 read_encoder.py [串口设备] [--save 文件名.csv]
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
        # 在macOS上，USB CDC通常是 /dev/cu.usbmodem*
        # 在Linux上，通常是 /dev/ttyACM*
        # 在Windows上，通常是 COM*
        if 'usbmodem' in port.device or 'ttyACM' in port.device:
            print(f"找到USB CDC设备: {port.device} ({port.description})")
            return port.device

    print("未找到USB CDC设备，显示所有可用串口:")
    for port in ports:
        print(f"  {port.device}: {port.description}")

    return None

def parse_encoder_line(line):
    """
    解析ENCODER行
    格式: ENCODER,timestamp,yaw_raw,pitch_raw,yaw_target,pitch_target
    """
    parts = line.strip().split(',')
    if len(parts) != 6 or parts[0] != 'ENCODER':
        return None

    try:
        data = {
            'timestamp': int(parts[1]),
            'yaw_raw': int(parts[2]),
            'pitch_raw': int(parts[3]),
            'yaw_target': float(parts[4]),
            'pitch_target': float(parts[5])
        }
        return data
    except (ValueError, IndexError):
        return None

def main():
    parser = argparse.ArgumentParser(description='读取云台编码器值')
    parser.add_argument('port', nargs='?', default=None,
                        help='串口设备 (例如: /dev/cu.usbmodem1234, 留空自动查找)')
    parser.add_argument('--save', metavar='FILE', default=None,
                        help='保存数据到CSV文件')
    parser.add_argument('--baud', type=int, default=115200,
                        help='波特率 (默认: 115200)')

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
        time.sleep(0.5)  # 等待串口稳定
        print(f"已连接！正在读取ENCODER数据... (按Ctrl+C退出)\n")
    except serial.SerialException as e:
        print(f"错误: 无法打开串口 {port}: {e}")
        sys.exit(1)

    # 打开CSV文件（如果需要）
    csv_file = None
    if args.save:
        csv_file = open(args.save, 'w')
        csv_file.write("system_time,timestamp_ms,yaw_raw,pitch_raw,yaw_target,pitch_target,yaw_error,pitch_error\n")
        print(f"数据将保存到: {args.save}\n")

    # 打印表头
    print(f"{'时间':<12} {'Yaw原始':>8} {'Yaw目标':>8} {'Yaw误差':>8} | "
          f"{'Pitch原始':>8} {'Pitch目标':>8} {'Pitch误差':>8}")
    print("-" * 80)

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

                    # 只处理ENCODER开头的行
                    if line.startswith('ENCODER'):
                        data = parse_encoder_line(line)
                        if data:
                            # 计算误差
                            yaw_error = data['yaw_target'] - data['yaw_raw']
                            pitch_error = data['pitch_target'] - data['pitch_raw']

                            # 处理环绕 (GM6020编码器范围0-8191)
                            if yaw_error > 4096:
                                yaw_error -= 8192
                            elif yaw_error < -4096:
                                yaw_error += 8192

                            if pitch_error > 4096:
                                pitch_error -= 8192
                            elif pitch_error < -4096:
                                pitch_error += 8192

                            # 格式化输出
                            timestamp_sec = data['timestamp'] / 1000.0
                            print(f"{timestamp_sec:>10.2f}s "
                                  f"{data['yaw_raw']:>8d} {data['yaw_target']:>8.1f} {yaw_error:>8.1f} | "
                                  f"{data['pitch_raw']:>8d} {data['pitch_target']:>8.1f} {pitch_error:>8.1f}")

                            # 保存到CSV
                            if csv_file:
                                system_time = datetime.now().isoformat()
                                csv_file.write(f"{system_time},"
                                               f"{data['timestamp']},"
                                               f"{data['yaw_raw']},"
                                               f"{data['pitch_raw']},"
                                               f"{data['yaw_target']:.2f},"
                                               f"{data['pitch_target']:.2f},"
                                               f"{yaw_error:.2f},"
                                               f"{pitch_error:.2f}\n")
                                csv_file.flush()
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

if __name__ == '__main__':
    main()
