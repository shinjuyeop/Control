# Deprecated can_lab path

The CAN lab implementation moved to `kaiev26_hil.bench` so Bench/SIL and
physical STM_TEST HIL tools live under one clearly separated package.

Preferred command after building the ROS workspace:

```bash
ros2 run kaiev26_hil can_sim --motor-iface vcan0 --steer-iface vcan1
```

The old command remains available through a compatibility wrapper:

```bash
python3 software/src/can_lab/teamkai_can_lab/teamkai_can_lab.py --motor-iface vcan0 --steer-iface vcan1
```
