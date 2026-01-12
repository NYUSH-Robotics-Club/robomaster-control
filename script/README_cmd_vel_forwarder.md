cmd_vel_forwarder README

Purpose
- Forward `cmd_vel` (ROS2 `geometry_msgs/Twist`) or CLI-specified vx,vy,wz to the STM32 via serial (USB CDC).
- Encodes frames using the project's radar protocol: [0xA5][0x5A][vx][vy][wz][crc8].

Dependencies
- Python 3
- `pyserial` (pip)
- Optional: ROS2 and `rclpy` when running in `--ros2` mode

Install

```bash
pip3 install pyserial
# If using ROS2 mode, install ROS2 following your Jetson distro instructions
```

Quick run examples

Non-ROS (periodic send):

```bash
python3 script/cmd_vel_forwarder.py --port /dev/ttyACM0 --baud 115200 --vx 0.5 --vy 0.0 --wz 0.0 --rate 20
```

ROS2 mode (subscribe to /cmd_vel):

```bash
# Make sure ROS2 environment is sourced (e.g. . /opt/ros/foxy/setup.bash)
python3 script/cmd_vel_forwarder.py --port /dev/ttyACM0 --baud 115200 --ros2 --topic /cmd_vel
```

Testing tips
- Use `script/test_radar_comm.py` locally to sanity-check frame formation and device reception.
- Monitor STM32 serial with `screen /dev/ttyACM0 115200` or similar (if firmware echoes or logs).

Safety
- Stop forwarding when testing switches back to RC or other control sources.
- Verify STM32 has `RadarComm_Task()` enabled and `MsgCenter` is dispatching prior to `CmdController_Task()`.
