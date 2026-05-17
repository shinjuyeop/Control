%% 0. CSV에서 실제 데이터 기반 전달함수 자동 추출
csv_path = '/home/junhyeon/workspace/Control/pid_gazebo/analysis/sils_bag_step4.csv';
data = readtable(csv_path);
step_time = max(data.t(data.setpoint_rpm == 0));
df_after = data(data.t > step_time, :);
t_rel = df_after.t - step_time;
steady_state = mean(df_after.odom_vx(end-50:end));
[max_val, max_idx] = max(df_after.odom_vx);
tp = t_rel(max_idx);
Mp = (max_val - steady_state) / steady_state;
zeta = -log(Mp) / sqrt(pi^2 + log(Mp)^2);
wn = pi / (tp * sqrt(1 - zeta^2));
input_mps = 30 * 2 * pi * 0.3 / 60;
K = steady_state / input_mps;
fprintf('K=%.4f, zeta=%.4f, wn=%.4f\n', K, zeta, wn);

%% 1. 2차 전달함수 자동 생성
G = tf(K * wn^2, [1, 2*zeta*wn, wn^2]);

%% 2. 보상 전 PID (wc 자동)
[C_before, info_before] = pidtune(G, 'PID');
fprintf('보상 전 → wc=%.2f rad/s, PM=%.1f deg\n', ...
    info_before.CrossoverFrequency, info_before.PhaseMargin);

%% 3. 보상 후 PID (응답시간 0.1초 기준 wc=10)
wc_target = 10;  % 응답시간 = 1/wc = 0.1초(수정하는 부분)
[C_after, info_after] = pidtune(G, 'PID', wc_target);
fprintf('보상 후 → wc=%.2f rad/s, PM=%.1f deg\n', ...
    info_after.CrossoverFrequency, info_after.PhaseMargin);

Kp = C_after.Kp
Ki = C_after.Ki
Kd = C_after.Kd

%% 4. 루프 게인 Bode Plot 비교
L_before = C_before * G;
L_after  = C_after  * G;

figure(1)
margin(L_before)
title('보상 전 Bode Plot (Loop Gain)')

figure(2)
margin(L_after)
title('보상 후 Bode Plot (Loop Gain)')

%% 5. 폐루프 스텝 응답 비교
figure(3)
step(feedback(L_before, 1))
title('보상 전 스텝 응답 (폐루프)')

figure(4)
step(feedback(L_after, 1))
title('보상 후 스텝 응답 (폐루프)')

%% 6. CSV 저장
save_dir = '/home/junhyeon/workspace/Control/pid_gazebo/pid_analysis';
mkdir(save_dir);
gains = [C_after.Kp, C_after.Ki, C_after.Kd];
writematrix(gains, fullfile(save_dir, 'pid_gains.csv'));
disp('save completion')