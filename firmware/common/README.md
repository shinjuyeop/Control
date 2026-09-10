# STM_A / STM_B runtime metrics

`runtime_metrics.c/.h`는 두 보드에서 공유하는 Cortex-M4 DWT CYCCNT 계측 모듈입니다.
`runtime_periodic.h`는 Control/Inputs/BTx/ATx/PerfMon의 absolute scheduling을
담당합니다. `.ioc` task 구성, FreeRTOS 설정, CAN/UART protocol, 기존 vehicle/status message는
변경하지 않습니다.

## Scheduling과 계측 구간

`RuntimePeriodic_Init()`의 최초 release phase를 기준으로 `release += period`인
fixed-phase timeline을 유지합니다. actual activation이나 completion 시점으로
re-anchor하지 않습니다. 완료 시 이미 도달하거나 지난 release는 period 단위로
건너뛰고, 완료 tick보다 엄격히 미래인 첫 scheduled release를 `vTaskDelayUntil()`로
기다립니다. 따라서 놓친 release를 catch-up burst로 실행하지 않습니다.

예를 들어 최초 release 100, period 20이면 timeline은 100/120/140/160/...입니다.
120 release를 127에 시작해 129에 완료해도 다음 release는 140입니다. 160 release를
205에 시작해 206에 완료하면 180/200을 건너뛰어 220을 기다립니다. 완료가 scheduled
release와 정확히 같아도 해당 release는 건너뜁니다. unsigned tick 차이와 덧셈으로
wrap을 처리하며, 기존 jitter와 동일하게 release부터 completion까지의 간격이
32-bit tick 범위의 절반(2^31 ticks) 미만임을 전제로 합니다. 한 tick 이내 지연은
구별할 수 없습니다.
이 정책은 RTOS activation 주기에 관한 것이며 CAN bus arbitration/ISR preemption을
포함한 실제 wire 송신 간격의 hard real-time 보장을 뜻하지 않습니다.

| RM ID | STM_A | STM_B | Activation 구간 |
|---|---|---|---|
| `RM_CONTROL` | Control | 없음 | 한 loop의 제어 처리; handshake 미완료 시 짧은 검사도 포함 |
| `RM_INPUTS` | Inputs | 없음 | ADC 8회 polling 및 GPIO/페달 계산 |
| `RM_UROS` | uROS | uROS | 연결 상태 머신 한 iteration; 명시적 5/10 ms delay 제외 |
| `RM_RX` | BRx | ARx | ring drain, timeout, recovery 검사; 1 ms delay 제외 |
| `RM_TX` | BTx | ATx | frame 생성과 blocking UART 송신; period 대기 제외 |
| `RM_BRAKE` | 없음 | Brake | 기존 ramp loop 처리; 5 ms delay 제외 |
| `RM_ESTOP` | 없음 | EStop | flags wait 복귀 후 safety 상태 처리; wait 및 최초 부팅 초기화 제외 |
| `RM_DISPLAY` | 없음 | Display | Dashboard CAN slot 처리; 기존 delay-until 제외 |
| `RM_PERFMON` | PerfMon | PerfMon | aggregate 처리; 1초 대기 제외 |

Control handshake 대기는 기존 10 ms이며 명령을 보내지 않습니다. 기존 handshake
분기의 periodic 초기화도 유지하며 fixed phase는 그 이후 제어 주기에 적용됩니다.
handshake 검사도 activation count에 포함되므로 이를 20 Hz command count로 해석하면
안 됩니다. Dashboard CAN은 기존 absolute
slot scheduling과 catch-up 동작을 유지합니다. Brake/EStop은 단순 fixed-period task로
바꾸지 않았습니다. 명시적 delay를 제외해도 UART/ADC/ROS 내부 blocking은 포함됩니다.

## Metric 정의

모든 task count/total/min/max는 boot 이후 누적입니다. 1 Hz는 통계 reset 주기가
아니라 snapshot 발행 주기입니다. `window_ms`는 직전 aggregate부터의 RTOS 경과 시간,
`uptime_ms`는 tick wrap을 확장한 boot 이후 시간입니다. rate는 두 snapshot의 count
차이와 시간 차이로 구합니다. Low task가 늦게 실행되면 window가 1초보다 길 수 있습니다.

- `activations`: Begin에 진입한 횟수. `completed`: End에 도달한 횟수.
- `timed_samples`: 유효한 cycle duration 횟수. average의 분모입니다.
- `wall_cycles_total/max`, `wall_us_total/max/average`: Begin→End의 observed activation
  wall time입니다. **Interrupt, preemption, 내부 blocking을 포함하며 formal WCET가
  아닙니다.** 측정 call 자체의 일부 overhead도 포함합니다. 개별 task wall time을
  더해서 CPU load로 해석하지 않습니다.
- `jitter_ticks_min/max`: actual activation tick − intended release tick.
  현재 1 kHz RTOS tick이므로 1 ms 해상도이며 sub-ms jitter는 보이지 않습니다.
  첫 release도 표본에 포함합니다. `jitter_samples=0`이면 미계측이지 jitter 0의 증명이
  아닙니다. RX/uROS/Brake/EStop은 nominal release가 없어 jitter/miss를 기록하지 않습니다.
- `deadline_misses`: completion tick이 원래 intended release + period 이상인
  activation 수. 늦게 시작해 deadline을 넘긴 경우도 포함하고 activation당 한 번만
  증가합니다. 놓친 모든 calendar slot의 개수나 safety timeout 횟수는 아닙니다.
- `stack_free_words`: FreeRTOS stack high-water mark, 생성 후 관측된 최소 여유 stack.
  단위는 32-bit word이며 bytes는 ×4입니다. 해당 시점의 free stack이나 overflow
  방지 보장이 아닙니다. `present=0`인 task slot은 해당 보드에서 미사용입니다.
- `free_heap_bytes`, `minimum_free_heap_bytes`: FreeRTOS heap_4의 현재/최소 여유량.
  linker의 static RAM 사용량과 별개이며 libc allocator/전체 RAM의 free 값이 아닙니다.
- `instrumentation_errors`: DWT 초기 동작 실패, task 생성 실패, wrap 판별이 불가능한
  긴 duration/dispatch 표본의 누락 횟수. 통신 오류 counter와 구별합니다.
- `cpu_load_valid=0`: 이번에는 scheduler/idle hook을 추가하지 않아 CPU load 미지원.
  `cpu_load_permille=0`을 0% CPU 사용량으로 해석하지 않습니다.

Hot path는 raw cycles, RTOS ticks와 integer 연산만 사용합니다. dynamic allocation,
printf, floating-point formatting, ROS publish가 없습니다. cycle→µs 변환은 snapshot에서
초기 `SystemCoreClock`으로 계산합니다. 현재 firmware의 고정 core clock을 전제로 하며
runtime clock 변경/STOP mode/debugger halt 구간의 시간 정확도는 보장하지 않습니다.

CYCCNT 차이는 unsigned subtraction으로 단일 32-bit wrap을 처리합니다. 170 MHz에서는
약 25.26초마다 wrap합니다. RTOS tick 경과가 이 한계에 접근한 wall-time 표본은 버리고
error를 올립니다. dispatch는 뒤늦게 도착한 ISR과 순서를 비교하므로 보수적으로 반 wrap
미만만 허용합니다. task의 누적 count/total은 64-bit, ISR/error counter는 32-bit입니다.
release 비교는 FreeRTOS 32-bit tick의 반 범위 미만 간격을 전제로 합니다. timestamp
값 자체는 cycle counter wrap마다 재사용되며 UTC/ROS 시간과 직접 비교할 수 없습니다.

## PerfMon / snapshot API

CubeMX 생성 PerfMon을 그대로 사용합니다: **Low / 256 words / Dynamic**, 약 1 Hz.
hot-path 직접 측정은 각 task와 ISR이 하고 PerfMon은 수집된 통계만 집계합니다.
stack scan과 정수 시간 변환은 critical section 밖에서 실행합니다. critical section은
counter update 또는 task 한 개의 고정 크기 record 복사에만 사용합니다. 고우선순위
task를 block하는 mutex나 scheduler suspension은 사용하지 않습니다.

큰 staging/publication snapshot은 static storage에 있으므로 PerfMon의 작은 stack에
올리지 않습니다. 각 record는 일관되게 복사되지만 모든 task가 정확히 같은 cycle에
동결된 global snapshot은 아닙니다. PerfMon 자신의 completed/wall metric은 직전
iteration까지 반영되고 activations는 현재 iteration을 포함할 수 있습니다.

`RuntimeMetrics_GetSnapshot(RuntimeMetricsSnapshot *out)`은 caller 소유 buffer에 복사합니다.
첫 snapshot 전 또는 publication 중/preemption으로 copy가 겹치면 `false`를 반환합니다.
이때 out을 버리고 다음 task iteration에서 다시 시도해야 합니다. 높은 priority에서
spin retry하면 낮은 priority writer를 굶길 수 있습니다. 성공한 복사본은 이후 publish가
일어나도 바뀌지 않으므로 uROS message 변환에 사용할 수 있습니다. pointer를
공유하지 말고, uROS에서도 큰 buffer는 static/caller 소유 저장소로 준비하십시오.

## STM_B E-stop latency

EXTI callback의 첫 C timestamp, flags wait에서 돌아온 task timestamp를 저장합니다.
기존 EXTI priority는 모두 5이며 같은 priority ISR끼리 nest하지 않습니다. ISR은
timestamp/counter를 저장한 다음 기존 flags notification을 수행합니다. pending 중
추가 edge는 `coalesced`에 세고 최초 pending callback 시각을 유지합니다.

`dispatch_cycles_max/us_max`는 최초 pending callback→task 처리 시작의 observed max,
`dispatch_samples`는 처리한 pending 묶음 수입니다. `last_isr_cycle`은 마지막 callback,
`last_task_cycle`은 마지막 task pass 시각이므로 두 last 필드를 임의로 빼서 paired
latency로 사용하면 안 됩니다. timeout만으로 깨어난 pass도 task 시각은 갱신하지만
pending ISR이 없으면 dispatch sample은 추가하지 않습니다. ISR이 task timestamp보다
뒤에 도착했다면 다음 pass까지 보존합니다. `RM_ESTOP`의 wall metric이 safety-state
처리 duration입니다.

**external physical edge→first ISR instruction의 true interrupt latency가 아닙니다.**
EXTI HAL handler와 callback prologue 이전 지연은 software timestamp에 포함되지
않습니다. 한 묶음의 각 edge에 대한 개별 event latency 보장도 아닙니다.

후속 board-only HIL 측정에서는 STM_TEST가 발생시키는 edge를 logic analyzer의 한
channel로, STM_B의 충돌 없는 spare GPIO를 다른 channel로 관측합니다. STM_B IRQ
handler의 earliest user hook 및 EStop 처리 시작에 marker를 넣어 edge→IRQ와
IRQ→task를 나눠 측정합니다. GPIO hook까지의 명령 overhead를 따로 산정해야 합니다.
대안은 동일 edge를 hardware timer input capture에 함께 배선하고 공통 timer로
timestamp하는 것입니다. spare pin/timer는 현재 핀맵과 CAN/PWM 배선을 확인한 후
선정해야 하며, 두 MCU의 독립 CYCCNT를 직접 빼지 않습니다.

이번에는 BRAKE_REQUEST→실제 PWM edge latency를 수집하지 않습니다. 기존 Brake는
5 ms loop, 20 ms ramp step, timer CCR/preload, 출력 pulse, 실제 servo 동작이 서로
다른 단계입니다. 단순 CCR write 시각을 물리 PWM/브레이크 적용 완료로 부르지 않습니다.
후속 측정에서 request marker와 STM_TEST의 PWM capture 또는 logic analyzer를 연결해
첫 변화와 ramp 완료를 각각 구분하십시오.

## 유지한 I/O 구조

BRx/ARx는 Receive-to-Idle circular DMA→ring→1 ms task polling을 유지합니다.
이 loop는 수신이 없을 때도 stale timeout 및 DMA restart를 평가합니다. notification
전환은 bounded timeout wake와 오류 callback wake를 함께 설계해야 하며, 현재 1 ms
polling 비용을 줄일 실측 근거가 없어 safety 경로 변경 이득보다 회귀 위험이 큽니다.
이번에는 기존 경로의 실제 비용을 먼저 계측합니다.

BTx/ATx도 blocking `HAL_UART_Transmit(..., 10u)`을 유지합니다. 특히 ATx safety frame의
local buffer는 함수 반환까지 살아 있습니다. 비동기 DMA로 바꾸려면 지속 buffer,
HAL_BUSY 정책, completion 소유권, timeout/abort/error recovery를 모두 마련해야 합니다.
DMA 설정이 있다는 이유만으로 치환하지 않습니다. 기존 수신 validity/CRC/sequence,
100 ms STM_B safety timeout, AUTO host 500 ms timeout은 변경하지 않았습니다.

## Host tests / ROS 연결

Control root에서 production module의 wrap, schedule overrun, snapshot publication,
E-stop dispatch/coalescing을 native로 확인할 수 있습니다. 이 테스트는 MCU timing이나
실제 ISR priority/preemption을 실측하지 않습니다.

```bash
cc -std=c11 -Wall -Wextra -Werror \
  -I firmware/STM_B/tests/mocks -I firmware/common \
  firmware/STM_B/tests/runtime_metrics_host_test.c firmware/common/runtime_metrics.c \
  -o /tmp/runtime_metrics_host_test
/tmp/runtime_metrics_host_test
```

`kaiev26_msgs/RuntimeStats`와 `TaskRuntimeStats`를 별도 진단 interface로 사용합니다.
기존 Drive/Brake/SteeringActuatorStatus와 VehicleState의 필드는 바꾸지 않습니다.

| Board | Topic | QoS | 발행 주체 |
|---|---|---|---|
| `BOARD_STM_A=1` | `/stm_a/runtime_stats` | best effort | STM_A uROS |
| `BOARD_STM_B=2` | `/stm_b/runtime_stats` | best effort | STM_B uROS |

`runtime_stats_ros.c`가 static caller-owned snapshot을 복사해 static ROS message로
변환합니다. `TaskRuntimeStats[9]`는 고정 배열이며 string, Header, 가변 sequence가
없습니다. task_id/slot은 위 RM ID 순서대로 0..8입니다. 미사용 slot도 task_id를
유지하고 present=false로 표시합니다. stack은 words에서 bytes로 변환합니다.

Count/total/extrema는 boot 이후 누적, `window_ms`는 PerfMon 집계 간격,
`uptime_ms`는 snapshot 시각입니다. Rate는 같은 board/boot의 counter 차이를
**snapshot uptime 차이**로 나눕니다. jitter_us는 signed 정수 µs지만 1 ms 해상도입니다.
wall_us는 preemption/interrupt/내부 blocking 포함 observed time이며 formal WCET가
아닙니다. cpu_load_valid는 현재 false이며, estop_valid는 B에서 timing과 dispatch
표본이 있을 때만 true입니다. A E-stop 필드는 false/zero입니다. E-stop 시간은
EXTI callback→task processing start이며 physical edge latency가 아닙니다.

uROS task만 publisher 생성/발행/정리를 수행하고 PerfMon은 집계만 합니다. 기존
command/status/ping 처리를 먼저 한 뒤 약 1 Hz로 최신 complete snapshot을 발행합니다.
발행 완료 후 1초를 기다려 지연 뒤 retry/catch-up burst를 만들지 않습니다. Snapshot
복사 실패는 건너뛰고, publish 실패는 오류를 해제한 뒤 기존 loop를 계속합니다.
진단 publisher 생성 실패도 기존 control entity 실행을 막지 않으며 다음 Agent
reconnect 때 재시도합니다. 런타임 message 저장소는 고정이며 발행 경로에 동적
할당을 추가하지 않습니다.

공식 `firmware/tools/regen_microros.sh`가 메시지 저장소의 clean commit을 사용해
A/B 모두 재생성합니다. Fixed message의 host CDR 크기는 796 bytes라 기존 512-byte
best-effort MTU에 들어가지 않습니다. generator의 `microxrcedds_client` 설정에
`UCLIENT_CUSTOM_TRANSPORT_MTU=1024`를 지정하며 generated header를 수동 수정하지
않습니다. Ubuntu GCC 10의 builtin stdint.h와 Newlib inttypes.h가 일관되게 동작하도록
workflow CFLAGS에 `-isystem /usr/include/newlib`도 지정합니다. 이 설정은 최근 rcutils
include 순서에서 드러난 PRId64 미정의 오류를 해결하며 MCU application의 compiler
flags나 generated typesupport를 직접 수정하지 않습니다.
LPUART1의 기존 DMA transport를 유지하고 MCU/Agent baud만 460800으로 맞춥니다. 이 transport의
write는 DMA 완료를 기다리므로 큰 diagnostic frame의 실제 status/callback 영향은
HIL에서 확인해야 합니다. Best effort는 ACK 대기를 피하지만 UART 전송시간을 없애지는
않습니다.

## 2026-09-10 구현 검증 범위

아래 수치와 HIL 관측은 최초 계측 구현(`ab60824`) 시점의 기록입니다.

시작점은 `Shin`, `d80e9a6e53fd367dea9390c6162d8ae235aebe11`, clean worktree입니다.
CubeMX 생성 baseline과 구현 후 모두 A/B Release clean build를 통과했습니다.

| Board | Flash baseline → 구현 후 | RAM baseline → 구현 후 |
|---|---:|---:|
| STM_A | 101,144 → 102,784 B (+1,640) | 96,600 → 99,880 B (+3,280) |
| STM_B | 99,744 → 101,488 B (+1,744) | 96,048 → 99,344 B (+3,296) |

각 build의 warning은 기존 `_gettimeofday` linker warning 1개로 동일합니다.
runtime metrics native test(ASan/UBSan 포함), UART protocol, Dashboard CAN protocol,
STM_TEST hil_logic test가 통과했고 Humble 환경에서 HIL protocol/transport/bridge
Python test 20개가 통과했습니다. MCU scheduling/preemption의 실측 증거는 아닙니다.

실행 중이던 HIL Web 서비스에서 명령 없이 5.031초간 26개 snapshot을 관측했습니다.
기존 펌웨어의 EZ command RX +99, feedback TX +200, drive status +199,
brake status +93이었고 EZ bus-off/TEC/REC는 0, PWM loss는 false였습니다.
페달/속도는 0, E-stop은 모두 해제, EZ RUN은 true, KEYA는 disabled인 기존 상태를
변경하지 않았습니다. A↔B UART는 status/failsafe 상태를 통한 간접 관측입니다.

새 firmware flash/reset, E-stop 주입, actuator 구동은 수행하지 않았습니다.
따라서 새 activation/jitter/deadline/heap/HWM/ISR-dispatch 값과 instrumentation
overhead, 새 firmware의 HIL safety regression은 미검증입니다. 물리 분리 상태와
flash 범위를 확인한 board-only bench에서 수행할 후속 항목입니다.

## RuntimeStats 전송속도

115200 baud 실보드 관측에서 RuntimeStats 약 796-byte 전송 때문에 /drive/status가
기존 약 40 Hz에서 36–37 Hz로 낮아지고 약 110 ms 수신 공백이 관측됐습니다.
DMA transport가 완료를 기다리는 동안 같은 uROS task가 진행하지 못하는 원인입니다.
기존 코드의 baud 우선 조정 방침에 따라 LPUART1만 460800으로 올리고 HIL/vehicle
Agent 기본값도 맞춥니다. USART1 A↔B, STM_TEST/Arduino serial, task 주기·priority,
DMA 구조는 유지합니다. 이전 firmware와 조합할 때는 `agent_baud:=115200`을 사용합니다.

460800-baud board-only HIL에서 runtime topic은 A 약 0.975 Hz, B 약 0.977 Hz,
정상 AUTO `/drive/status` 직접 subscriber는 약 41.7–42.3 Hz였습니다. `ros2 topic hz`
window도 약 40–44 Hz로 관측됐습니다. 115200에서 나타난 약 110 ms 공백은 정상
구간 최대 약 49 ms로 줄었습니다. 목표 50 Hz와의 나머지 차이는 기존 uROS loop의
관측 결과이며 scheduler는 변경하지 않았습니다. 65초 정상 관측에서 periodic
activation rate, error/miss, stack/heap 및 E-stop/timeout 회귀를 확인했습니다.

Host↔MCU의 baud는 firmware와 함께 배포해야 합니다. 현재 HIL은 기존 launch 구성에
`agent_baud:=460800`을 적용한 user service `kaiev26-hil-runtime-ros.service`로 실행합니다.
해당 bench를 종료할 때는 `systemctl --user stop kaiev26-hil-runtime-ros.service`를
사용합니다. 다시 수동 launch하기 전에 같은 장치를 소유한 이 서비스를 먼저 종료합니다.
