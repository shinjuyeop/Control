% pid_tuning.m
% 전달함수 기반 PID 자동 튜닝 스크립트

%% 1. 전달함수 정의
K = 0.96;
tau = 0.9;
G = tf(K, [tau 1]);

%% 2. PID 자동 튜닝
C = pidtune(G, 'PID');
Kp = C.Kp
Ki = C.Ki
Kd = C.Kd

%% 3. 스텝 응답 시각화
figure(1)
step(feedback(G, 1))
title('no pid')

figure(2)
step(feedback(C*G, 1))
title('after pid')

%% 4. CSV 저장
save_dir = '/home/junhyeon/workspace/Control/pid_gazebo/pid_analysis';
mkdir(save_dir);
gains = [C.Kp, C.Ki, C.Kd];
writematrix(gains, fullfile(save_dir, 'pid_gains.csv'));
disp('save completion')