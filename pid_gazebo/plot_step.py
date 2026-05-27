import pandas as pd
import matplotlib.pyplot as plt
import glob
import os

files = sorted(glob.glob('analysis/sils_bag_run_*.csv'))
rpm_map = {1:10, 2:10, 3:20, 4:20, 5:30, 6:30, 7:40, 8:40, 9:50, 10:50}

os.makedirs('plots', exist_ok=True)

for i, path in enumerate(files):
    run_id = i + 1
    df = pd.read_csv(path)
    rpm = rpm_map[run_id]
    target_mps = rpm * 2 * 3.14159 * 0.3 / 60

    step_time = df[df['setpoint_rpm'] == 0]['t'].max()
    df['t_rel'] = df['t'] - step_time

    fig, ax = plt.subplots(figsize=(10, 5))
    ax.plot(df['t_rel'].values, df['odom_vx'].values, label='odom_vx', color='blue', linewidth=1.5)
    ax.plot(df['t_rel'].values, df['cmd_linear_x'].values, label='cmd_linear_x', color='red', linewidth=1, linestyle='--')
    ax.axhline(y=target_mps, color='green', linestyle=':', linewidth=1.5, label=f'target {target_mps:.3f} m/s')
    ax.axvline(x=0, color='gray', linestyle='--', linewidth=1, label='step input')
    ax.set_title(f'Step Response — run_{run_id} | RPM={rpm}')
    ax.set_xlabel('time (s, 0=step input)')
    ax.set_ylabel('velocity (m/s)')
    ax.legend(fontsize=9)
    ax.grid(True, alpha=0.3)

    save_path = f'plots/step_response_run_{run_id}_rpm{rpm}.png'
    plt.savefig(save_path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f"저장: {save_path}")

print("완료!")