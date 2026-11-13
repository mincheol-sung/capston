%% WP1 — 메카넘 기구학 기초  [담당: 석원]
% 목표: 몸체속도[vx;vy;w] <-> 4바퀴 각속도 변환 함수 만들고 검증.
% 실측 없이 바로 실행됨. CAD에서 r,lx,ly 만 채우면 끝.
% (WP4 스타터의 body2wheel/wheel2body 와 같은 함수 = 정답 레퍼런스)
clear; clc; close all;

%% 0. CAD에서 채울 값 (TODO)
r  = 0.05;   % TODO 바퀴 반경 [m]
lx = 0.15;   % TODO 중심->바퀴 세로거리 [m]
ly = 0.15;   % TODO 중심->바퀴 가로거리 [m]

%% 1. 역기구학 행렬 (몸체속도 -> 바퀴)
L = lx + ly;
M = (1/r) * [ 1 -1 -L;
              1  1  L;
              1  1 -L;
              1 -1  L];      % 행 하나 = 바퀴 하나
disp('역기구학 행렬 M ='); disp(M);

%% 2. 왕복 테스트: wheel2body(body2wheel(v)) ≈ v 여야 함
v_test = [0.4; 0.2; 0.3];               % 임의 몸체속도
w      = body2wheel(v_test, r, lx, ly); % -> 4바퀴
v_back = wheel2body(w, r, lx, ly);      % -> 다시 몸체속도
fprintf('왕복 오차 = %.2e (1e-9 미만이면 OK)\n', norm(v_test - v_back));

%% 3. 부호 검증: 순수 전진/측면/회전 시 바퀴 방향 확인
cases = {[1;0;0],'전진 vx'; [0;1;0],'측면 vy'; [0;0;1],'회전 w'};
fprintf('\n[부호 검증] 각 모드에서 바퀴 방향(+/-):\n');
for i=1:3
    wi = body2wheel(cases{i,1}, r, lx, ly);
    fprintf('  %-8s : [%+d %+d %+d %+d]\n', cases{i,2}, sign(wi));
end
% 실기 확인: 실제 바퀴에 저속 명령 넣어 위 부호대로 도는지 육안 검증(제일 흔한 버그!)

%% 4. 시각화: 전진할 때 바퀴 속도 막대
figure('Name','WP1 kinematics');
subplot(1,2,1);
  bar(body2wheel([0.4;0;0], r,lx,ly)); title('전진(vx=0.4)'); xlabel('wheel'); ylabel('rad/s'); grid on;
subplot(1,2,2);
  bar(body2wheel([0;0;1], r,lx,ly)); title('회전(w=1)'); xlabel('wheel'); ylabel('rad/s'); grid on;

%% ===== local functions =====
function W = body2wheel(v,r,lx,ly)
    L=lx+ly;
    M=(1/r)*[1 -1 -L; 1 1 L; 1 1 -L; 1 -1 L];
    W=M*v;
end
function v = wheel2body(W,r,lx,ly)
    L=lx+ly;
    M=(1/r)*[1 -1 -L; 1 1 L; 1 1 -L; 1 -1 L];
    v=pinv(M)*W;   % 4->3 최소자승(정기구학)
end
