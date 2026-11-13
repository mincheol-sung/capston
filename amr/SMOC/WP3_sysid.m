%% WP3 — 축레벨 속도루프 1차 모델 식별 v2  [담당: 준서]
%  모델:  v(k+1) = a*v(k) + b*u(k-d)      (d = 추가 지연 샘플수)
%  실기 인터페이스: 명령 /cmd_vel(twist), 실측 /odom(몸체속도)
%
%  v1 대비 추가된 것
%   (1) procest('P1D') 로 dead time Td 진단 → 지연 d 자동 선택
%   (2) arx 로 이산 a,b 직접 식별 (System ID Toolbox 없으면 최소자승 폴백)
%   (3) 1스텝예측 VAF + 무한구간 시뮬레이션 fit% 를 둘 다 출력
%       ★ WP3 완료기준(fit>=85%)은 "시뮬레이션 fit" 으로 판정할 것
%   (4) 축 3개 일괄 처리 + WP4 axModel 용 결과표 자동 출력
clear; clc; close all;

%% ===== 0. 설정 =====
USE_CSV  = false;          % true: 실측 wp2_log.csv / false: 합성데이터
CSV_PATH = 'wp2_log.csv';
AXES     = [0 1 2];        % 0=vx, 1=vy, 2=w  (CSV의 active 열과 매칭)
Ts       = 0.02;           % 50Hz (로거 RATE_HZ와 반드시 일치)
MAXDELAY = 3;              % 지연 후보 스캔 범위 [샘플]
FIT_MIN  = 85;             % 완료기준 (시뮬레이션 fit%)

hasIdent = license('test','Identification_Toolbox') && ~isempty(ver('ident'));
if hasIdent
    fprintf('System Identification Toolbox: 사용 가능 (procest/arx 실행)\n\n');
else
    fprintf('System Identification Toolbox: 없음 → 최소자승 폴백으로 진행\n\n');
end
if ~USE_CSV, AXES = 0; end     % 합성데이터는 단일 축만

axName = ["vx","vy","w"];
Res = table('Size',[0 7], ...
    'VariableTypes',{'string','double','double','double','double','double','double'}, ...
    'VariableNames',{'axis','a','b','dcgain','delay_smp','VAF_1step','fit_sim'});

%% ===== 1~5. 축별 루프 =====
for ax = AXES
    %% 1. 데이터 준비
    if USE_CSV
        % CSV열: t_s,active,cvx,cvy,cw,ovx,ovy,ow,w0..w3
        T = readtable(CSV_PATH);
        rows = T.active == ax;
        if ~any(rows)
            warning('축 %s: 데이터 없음, 건너뜀', axName(ax+1)); continue;
        end
        cmdCols = {'cvx','cvy','cw'};  odoCols = {'ovx','ovy','ow'};
        u = T.(cmdCols{ax+1})(rows);
        v = T.(odoCols{ax+1})(rows);
        t = T.t_s(rows) - T.t_s(find(rows,1));
        fprintf('===== 축 %s : CSV %d 샘플 =====\n', axName(ax+1), numel(u));
    else
        a_true=0.6; b_true=0.3; d_true=1; N=200;      % d_true로 지연 검출 테스트
        u=zeros(N,1); u(11:end)=1.0;
        v=zeros(N,1);
        for k=d_true+1:N-1
            v(k+1)=a_true*v(k)+b_true*u(k-d_true)+0.002*randn;
        end
        t=(0:N-1)'*Ts;
        fprintf('===== 합성데이터 (a=%.2f b=%.2f d=%d) =====\n',a_true,b_true,d_true);
    end

    %% 2. 지연 진단 — procest('P1D')
    Td_est = NaN;
    if hasIdent
        dat = iddata(v, u, Ts);
        try
            sysP   = procest(dat, 'P1D');       % Kp/(1+Tp1*s)*exp(-Td*s)
            Td_est = sysP.Td;
            fprintf('  procest P1D : Kp=%.4f  Tp1=%.4f s  Td=%.4f s (= %.2f 샘플)\n', ...
                    sysP.Kp, sysP.Tp1, Td_est, Td_est/Ts);
        catch ME
            fprintf('  procest 실패(%s) → 지연은 스캔으로만 추정\n', ME.identifier);
        end
    end

    %% 3. 지연 d 스캔 + 최소자승 식별
    best = struct('fitS',-inf);
    for d = 0:MAXDELAY
        S = fitFirstOrder(u, v, d);
        fprintf('  d=%d : a=%7.4f b=%7.4f | VAF(1step)=%6.2f%%  fit(sim)=%6.2f%%\n', ...
                d, S.a, S.b, S.fit1, S.fitS);
        if S.fitS > best.fitS, best = S; best.d = d; end
    end
    d = best.d;  a = best.a;  b = best.b;
    fprintf('  → 선택: d=%d (시뮬 fit 최대)\n', d);
    if ~isnan(Td_est) && abs(Td_est/Ts - d) > 1
        warning('procest Td(%.2f 샘플)와 스캔 결과 d=%d 불일치 → 데이터 품질 점검', Td_est/Ts, d);
    end

    %% 4. arx 교차확인 (툴박스 있을 때)
    if hasIdent
        try
            m = arx(dat, [1 1 d+1]);            % na=1, nb=1, nk=d+1
            a_arx = -m.A(2);  b_arx = m.B(d+2);
            [~, fitArx] = compare(dat, m);
            if iscell(fitArx), fitArx = fitArx{1}; end
            fitArx = mean(fitArx(:));
            fprintf('  arx 교차확인 : a=%.4f b=%.4f  compare fit=%.2f%%\n', ...
                    a_arx, b_arx, fitArx);
            if max(abs([a_arx-a, b_arx-b])) > 1e-3
                fprintf('    (LS와 미세 차이는 정상 — arx는 예측오차 기준 최적화)\n');
            end
            a = a_arx;  b = b_arx;              % 툴박스 결과를 정본으로
            best.fitS = fitArx;
        catch ME
            fprintf('  arx 실패(%s) → 최소자승 결과 사용\n', ME.identifier);
        end
    end

    %% 5. 판정 + 그래프
    K = b/(1-a);
    fprintf('  ★ 결과: a=%.4f  b=%.4f  정상이득 b/(1-a)=%.4f  fit(sim)=%.2f%%\n', ...
            a, b, K, best.fitS);
    if best.fitS < FIT_MIN
        warning('fit %.1f%% < %d%% : qpps 스케일 / 포화구간 / 2차 동특성 여부 점검', ...
                best.fitS, FIT_MIN);
    end
    if abs(a) >= 1
        warning('|a|>=1 : 불안정 모델. 데이터에 드리프트/적분 성분이 섞였는지 확인');
    end

    figure('Name', sprintf('WP3 sysID - %s', axName(ax+1)));
    subplot(2,1,1);
    plot(t, v, 'b', 'LineWidth', 1.3); hold on;
    plot(t, best.vsim, 'r--', 'LineWidth', 1.3);
    plot(t, best.vpred, 'g:', 'LineWidth', 1.0);
    legend('측정(odom)', sprintf('모델 시뮬 (fit %.1f%%)',best.fitS), ...
           sprintf('1스텝 예측 (VAF %.1f%%)',best.fit1), 'Location','best');
    ylabel('velocity'); grid on;
    title(sprintf('축 %s :  a=%.3f  b=%.3f  d=%d 샘플', axName(ax+1), a, b, d));
    subplot(2,1,2);
    stairs(t, u, 'k', 'LineWidth', 1.0); grid on;
    xlabel('t [s]'); ylabel('cmd\_vel'); title('입력');

    Res = [Res; {axName(ax+1), a, b, K, d, best.fit1, best.fitS}]; %#ok<AGROW>
    fprintf('\n');
end

%% ===== 6. WP4 인계 =====
fprintf('========== WP3 결과표 (WP4 axModel 입력) ==========\n');
disp(Res);
for i = 1:height(Res)
    fprintf('axModel(%d).a = %.6f;  axModel(%d).b = %.6f;   %% %s\n', ...
            i, Res.a(i), i, Res.b(i), Res.axis(i));
end
if any(Res.fit_sim < FIT_MIN)
    fprintf('\n[경고] fit<%d%% 인 축이 있음 → WP4 진행 전 WP2 데이터 재수집 검토\n', FIT_MIN);
end
if height(Res) == 3
    spread = max(Res.a) - min(Res.a);
    fprintf('\n축간 a 편차 = %.4f  (0.1 이상이면 축별 튜닝 필요 신호)\n', spread);
end

%% ===== 로컬 함수 =====
function S = fitFirstOrder(u, v, d)
% v(k+1) = a*v(k) + b*u(k-d) 최소자승 + 1스텝예측/시뮬레이션 적합도
N   = numel(v);
idx = (d+1):(N-1);
Phi = [v(idx), u(idx-d)];
Y   = v(idx+1);
th  = Phi\Y;
S.a = th(1);  S.b = th(2);

% 1스텝 예측 (측정값을 매 스텝 넣어줌 → 낙관적)
yhat  = Phi*th;
S.fit1 = 100*(1 - var(Y-yhat)/var(Y));                 % VAF
S.vpred = nan(N,1);  S.vpred(idx+1) = yhat;

% 무한구간 시뮬레이션 (입력만 사용 → 실제 모델 품질)
vs = nan(N,1);  vs(1:d+1) = v(1:d+1);
for k = idx
    vs(k+1) = S.a*vs(k) + S.b*u(k-d);
end
e = v(idx+1) - vs(idx+1);
S.fitS = 100*(1 - norm(e)/norm(v(idx+1)-mean(v(idx+1))));   % NRMSE fit%
S.vsim = vs;
end