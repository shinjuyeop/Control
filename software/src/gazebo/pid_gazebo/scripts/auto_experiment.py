#!/usr/bin/env python3
"""
완전 자동화 실험 스크립트
- RPM을 바꿔가며 10회 주행 데이터 자동 녹화
- 각 회차마다 analyze_bag.py 자동 실행 → CSV 생성
- MATLAB 튜닝 스크립트 자동 실행 → PID 파라미터 추출
"""

import subprocess
import time
import os
import signal
import shutil
from pathlib import Path

# 실험 설정
rpm_list = [10, 10, 20, 20, 30, 30, 40, 40, 50, 50]
record_duration = 15  # 녹화 시간 (초)
wait_between = 3      # 회차 간 대기 시간 (초)
zero_duration = 3     # RPM=0 유지 시간 (초)

SCRIPT_DIR = Path(__file__).resolve().parent
WORK_DIR = SCRIPT_DIR.parent
MATLAB_SCRIPT = WORK_DIR / 'matlab' / 'auto_pid_tuning.m'
PID_GAINS_PATH = WORK_DIR / 'config' / 'pid_gains.csv'
PID_GAINS_DEFAULT = '0.5,0.05,0.02'  # 데이터 수집용 안정적인 기본값
BAG_DIR = WORK_DIR / 'bags'
ANALYSIS_DIR = WORK_DIR / 'analysis'

print("=== 완전 자동화 실험 시작 ===")
print(f"총 {len(rpm_list)}회 실험 | 각 {record_duration}초 녹화")
print(f"RPM 목록: {rpm_list}")

# 잔존 노드 종료
subprocess.run(['pkill', '-f', 'setpoint_publisher'], capture_output=True)
print("잔존 setpoint_publisher 종료 완료!")

# 기존 실험 데이터 삭제
print("\n기존 실험 데이터 삭제 중...")
BAG_DIR.mkdir(exist_ok=True)
ANALYSIS_DIR.mkdir(exist_ok=True)
PID_GAINS_PATH.parent.mkdir(exist_ok=True)
for folder in BAG_DIR.glob('sils_bag_run_*'):
    shutil.rmtree(folder)
for csv_path in ANALYSIS_DIR.glob('sils_bag_run_*.csv'):
    csv_path.unlink()
result_path = ANALYSIS_DIR / 'pid_results.csv'
if result_path.exists():
    result_path.unlink()
print("기존 실험 데이터 삭제 완료!")

for i, rpm in enumerate(rpm_list):
    print(f"\n{'='*40}")
    print(f"=== 회차 {i+1}/{len(rpm_list)} | RPM={rpm} ===")
    print(f"{'='*40}")

    # 1. pid_gains.csv 기본값으로 리셋 (안정적인 데이터 수집용)
    with open(PID_GAINS_PATH, 'w') as f:
        f.write(PID_GAINS_DEFAULT)
    print(f"pid_gains.csv 기본값으로 리셋: {PID_GAINS_DEFAULT}")

    # 2. current_rpm.txt에 rpm 값 저장 (MATLAB에서 읽어감)
    with open(WORK_DIR / 'current_rpm.txt', 'w') as f:
        f.write(str(rpm))

    # 3. 회차 번호로 bag 이름 생성 (절대경로 기준 폴더명)
    bag_name = f'sils_bag_run_{i+1}'
    bag_abs_path = BAG_DIR / bag_name
    env = os.environ.copy()
    env['PID_GAZEBO_BAG_DIR'] = str(BAG_DIR)
    env['PID_GAZEBO_BAG_NAME'] = bag_name

    # 4. launch 실행 (브릿지 + pid노드 + rosbag만)
    proc = subprocess.Popen([
        'ros2', 'launch', 'pid_gazebo', 'sils.launch.py'
    ], cwd=WORK_DIR, env=env)

    # 5. RPM=0 지속 발행 (3초간 → 약 30개 데이터)
    print(f"RPM=0 정지 중... {zero_duration}초 대기")
    proc_zero = subprocess.Popen([
        'ros2', 'topic', 'pub', '--rate', '10',
        '/setpoint/rpm', 'std_msgs/msg/Float64', '{data: 0.0}'
    ])
    time.sleep(zero_duration)
    proc_zero.terminate()
    proc_zero.wait()

    # 6. 목표 RPM으로 스텝 입력 후 지속 발행 (15초간 → 약 150개 데이터)
    print(f"RPM={rpm} 스텝 입력! {record_duration}초 유지")
    proc_step = subprocess.Popen([
        'ros2', 'topic', 'pub', '--rate', '10',
        '/setpoint/rpm', 'std_msgs/msg/Float64',
        f'{{data: {float(rpm)}}}'
    ])

    # 7. 녹화 대기
    for t in range(record_duration, 0, -1):
        print(f"녹화 중... {t}초 남음", end='\r')
        time.sleep(1)

    proc_step.terminate()
    proc_step.wait()

    # 8. SIGINT로 부드럽게 종료 (bag 파일 정상 저장)
    proc.send_signal(signal.SIGINT)
    proc.wait(timeout=10)
    print(f"\n녹화 완료!")

    # bag 파일 저장 완료 대기
    time.sleep(5)

    # 9. analyze_bag.py 자동 실행 → 절대경로로 bag 전달
    print("bag 분석 중...")
    subprocess.run(
        ['python3', str(SCRIPT_DIR / 'analyze_bag.py'), str(bag_abs_path)],
        cwd=WORK_DIR,
        check=True,
    )
    print("CSV 생성 완료!")

    # 10. MATLAB 튜닝 스크립트 자동 실행
    print("MATLAB PID 튜닝 중...")
    subprocess.run([
        'matlab', '-nodisplay', '-nosplash', '-nodesktop',
        '-r', f"run('{MATLAB_SCRIPT}'); exit;"
    ], check=True)
    print(f"=== 회차 {i+1} PID 추출 완료 ===")

    if i < len(rpm_list) - 1:
        print(f"{wait_between}초 후 다음 회차 시작...")
        time.sleep(wait_between)

print("\n=== 전체 실험 완료! ===")
print(f"{ANALYSIS_DIR}/pid_results.csv")
