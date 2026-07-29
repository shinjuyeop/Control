# Control firmware

STM_A는 종방향 모터와 차량 입력을 담당하고, STM_B는 물리 E-stop과 브레이크
서보를 담당합니다. 두 보드는 USART1으로 차량 상태와 최종 브레이크 요청을 서로
교환하며, 각 보드는 LPUART1 115200 baud로 별도의 micro-ROS Agent에 연결됩니다.

## 명령과 안전 경로

판단·motion control 노드는 `kaiev26_msgs/msg/ActuatorCommand`를
`/vehicle/command`로 발행합니다.

- STM_A는 `speed_target_mps`와 `brake_engage`를 직접 받습니다.
- STM_B는 같은 메시지의 `brake_engage`를 독립적으로 받습니다.
- STM_B는 물리 모드 상태를 STM_A의 UART vehicle-status frame에서 받습니다.
- STM_B는 물리 E-stop과 AUTO의 `brake_engage`를 OR한 최종
  `BRAKE_REQUEST`로 브레이크 서보를 움직이고, 같은 값을 UART safety-status
  frame으로 STM_A에 보냅니다.
- STM_A는 AUTO의 직접 `brake_engage`, STM_B의 `BRAKE_REQUEST`, STM_B UART
  timeout 중 하나라도 참이면 종방향 모터에 정지·HALTED 명령을 보냅니다.

브레이크 서보를 직접 구동하는 보드는 STM_B뿐입니다. STM_A는 브레이크 서보를
구동하지 않습니다.

MANUAL에서는 Host의 `brake_engage`가 브레이크 서보와 수동 페달 구동을 정지시키지
않습니다. 물리 E-stop만 MANUAL의 브레이크 서보를 작동시킵니다. 따라서 AUTO에서
Host 브레이크가 걸린 상태로 MANUAL로 전환하면, upstream이
`brake_engage=false`를 다시 보내지 않아도 Host 원인의 브레이크는 해제됩니다.

STM_A의 종방향 애플리케이션 명령 주기는 50 ms, 즉 20 Hz입니다. 이것은
EZkontrol 모터 드라이버 내부의 전류 제어 주기가 아니라 STM_A가 속도 또는 토크
목표를 갱신하는 주기입니다.

## EZkontrol CAN

STM_A가 EZkontrol 모터 드라이버와 CAN으로 통신합니다.

- STM_A는 모터 드라이버의 handshake 요청을 받고 ACK 전송에 성공하기 전까지
  traction 명령을 보내지 않습니다.
- STM_A는 EZkontrol status Part I과 Part II를 모두 수신합니다.
- Part I은 RPM, phase current와 DC voltage를 갱신합니다.
- Part II는 RUN 상태, control mode, controller·motor temperature와 fault bits를
  갱신합니다.
- STM_A는 이 값을 `/drive/status`의 `DriveActuatorStatus`로 발행합니다.

STM_B는 EZkontrol CAN Part I/II를 수신하지 않습니다. STM_B가 표시하는 속도,
기어와 모드는 STM_A가 USART1 vehicle-status frame으로 보낸 값입니다.

## STM 빌드

두 프로젝트의 CMake preset은 STM32CubeCLT 1.20.0의 ARM toolchain을 사용합니다.

```bash
cmake -S firmware/STM_A --preset Release
cmake --build firmware/STM_A/build/Release

cmake -S firmware/STM_B --preset Release
cmake --build firmware/STM_B/build/Release
```

생성되는 ELF·HEX·BIN을 플래시하는 절차는 사용하는 ST-Link 환경에 맞게 수행합니다.
플래시 전에는 대상 보드가 STM_A인지 STM_B인지 확인하십시오.

## micro-ROS Agent 빌드

`firmware/STM_A`와 `firmware/STM_B`에는 `COLCON_IGNORE`가 있으므로, 다음 명령은
vendored Agent와 `micro_ros_msgs`만 ROS 패키지로 빌드합니다.

```bash
source /opt/ros/humble/setup.bash

colcon build \
  --base-paths firmware \
  --packages-select micro_ros_msgs micro_ros_agent

source install/setup.bash
```

vendored upstream revision과 로컬 빌드 수정은
`firmware/micro_ros_agent/UPSTREAM.md`에 기록되어 있습니다.

## Agent 실행

두 보드를 동시에 연결하려면 서로 다른 serial device에 Agent를 하나씩 실행합니다.
아래 device 이름은 예시이므로 `udevadm info` 등으로 실제 보드를 식별한 뒤
대입하십시오.

터미널 1, STM_A:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run micro_ros_agent micro_ros_agent serial \
  --dev /dev/serial/by-id/<stm-a-device> -b 115200
```

터미널 2, STM_B:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run micro_ros_agent micro_ros_agent serial \
  --dev /dev/serial/by-id/<stm-b-device> -b 115200
```

연결 후에는 node 이름뿐 아니라 타입과 실제 endpoint를 함께 확인합니다.

```bash
ros2 node list
ros2 topic info -v /vehicle/command
ros2 topic info -v /drive/status
ros2 topic info -v /brake/status
ros2 topic echo /drive/status
ros2 topic echo /brake/status
```

정상 상태에서는 `/stm_a_node`와 `/stm_b_node`가 보이고, 두 보드 모두
`/vehicle/command`를 구독합니다. STM_A는 `/drive/status`, STM_B는
`/brake/status`를 발행합니다.
