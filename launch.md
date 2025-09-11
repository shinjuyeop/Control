# 빠른 시작 (실행 가이드)

1. Control 선택 후 클린 빌드

```bash
cd ~/KAI_ws

# Control 아래 패키지 이름들 확인
PKGS=$(colcon list --base-paths src/Control --names-only)

# 해당 패키지들만 삭제
rm -rf log
for p in $PKGS; do
  rm -rf build/$p install/$p
done

# Control 다시 빌드
colcon build --base-paths src/Control --symlink-install

cd src/Control
```

2. 캔 설정

- 종방향

```bash
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 250000
sudo ip link set can0 up

candump can0
```

- 횡방향

```bash
sudo ip link set can1 down
sudo ip link set can1 type can bitrate 250000
sudo ip link set can1 up

candump can1
```

3. 런치 실행 (퍼블리셔는 제외)

- 종방향 제어

```bash
ros2 launch control_launch longitudinal.launch.py
```

- 횡방향 제어

```bash
ros2 launch control_launch lateral.launch.py
```

4. 필요시 퍼블리셔 노드 실행

```bash
ros2 run motor_control_cpp speed_publisher   # 250rpm

# 속도 조정
# ros2 run motor_control_cpp speed_publisher --ros-args -p rpm:=200
```

```bash
ros2 run steering_control steering_command_publisher_node
```

***종방향 없이 횡방향만 실행시켜도 throttle_serial_node는 실행해야 함 (스위치 데이터)***

```bash
ros2 run throttle_serial_py throttle_serial_node
```

5. 모니터 노드 (/ctrl/speed, /ctrl/steer 확인)

```bash
# /ctrl/speed
ros2 run speed_monitor_cpp speed_monitor_node
```

```bash
# /ctrl/steer
ros2 run steering_monitor_cpp steering_monitor_node
```
