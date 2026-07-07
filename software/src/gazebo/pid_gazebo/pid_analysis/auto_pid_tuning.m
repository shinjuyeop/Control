%% 0. CSV에서 실제 데이터 기반 전달함수 자동 추출
% current_rpm.txt에서 rpm 자동 읽기
script_dir = fileparts(mfilename('fullpath'));
package_dir = fileparts(script_dir);
rpm_file = fullfile(package_dir, 'current_rpm.txt');
rpm_input = str2double(fileread(rpm_file));
fprintf('현재 RPM: %.1f\n', rpm_input);
% 가장 최근 sils_bag_run_* CSV 자동 선택
analysis_dir = fullfile(package_dir, 'analysis');
csv_files = dir(fullfile(analysis_dir, 'sils_bag_run_*.csv'));  % ← 패턴 변경
if isempty(csv_files)
    error('sils_bag_run_*.csv 파일 없음');
end
[~, idx] = max([csv_files.datenum]);
csv_path = fullfile(analysis_dir, csv_files(idx).name);
fprintf('읽는 파일: %s\n', csv_path);
data = readtable(csv_path);
step_time = max(data.t(data.setpoint_rpm == 0));
df_after = data(data.t > step_time, :);
t_rel = df_after.t - step_time;
% 전체 데이터의 마지막 10% 사용
n = max(1, floor(height(df_after) * 0.1));
steady_state = mean(df_after.odom_vx(end-n+1:end));
[max_val, max_idx] = max(df_after.odom_vx);
tp = t_rel(max_idx);
Mp = (max_val - steady_state) / steady_state;
zeta = -log(Mp) / sqrt(pi^2 + log(Mp)^2);
wn = pi / (tp * sqrt(1 - zeta^2));
input_mps = rpm_input * 2 * pi * 0.3 / 60;
K = steady_state / input_mps;
fprintf('K=%.4f, zeta=%.4f, wn=%.4f\n', K, zeta, wn);
%% 1. 2차 전달함수 자동 생성
G = tf(K * wn^2, [1, 2*zeta*wn, wn^2]);
%% 2. 보상 전 PID (wc 자동)
[C_before, info_before] = pidtune(G, 'PID');
fprintf('보상 전 → wc=%.2f rad/s, PM=%.1f deg\n', ...
    info_before.CrossoverFrequency, info_before.PhaseMargin);
%% 3. 보상 후 PID (응답시간 0.1초 기준 wc=10)
wc_target = 10;  % 응답시간 = 1/wc = 0.1초
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
save_dir = script_dir;
mkdir(save_dir);
gains = [C_after.Kp, C_after.Ki, C_after.Kd];
writematrix(gains, fullfile(save_dir, 'pid_gains.csv'));
disp('save completion')
%% 7. 결과 누적 저장
result_path = fullfile(save_dir, 'pid_results.csv');
% 헤더 생성 (처음 한 번만)
if ~isfile(result_path)
    header = {'rpm', 'K', 'zeta', 'wn', 'wc', 'PM', 'Kp', 'Ki', 'Kd'};
    writecell(header, result_path);
end
% 결과 누적
result = {rpm_input, K, zeta, wn, wc_target, info_after.PhaseMargin, C_after.Kp, C_after.Ki, C_after.Kd};
writecell(result, result_path, 'WriteMode', 'append');
disp('결과 누적 저장 완료!')
