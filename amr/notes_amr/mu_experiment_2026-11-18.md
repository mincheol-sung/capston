# 2026-11-18 바닥 μ̂ · 바퀴별 토크 상한 실험 — 실행 순서

NUC `smoc@192.168.0.132`, 팀 체크아웃 `~/capston/capston` (smc a48d513) 의 `amr…` 명령을 그대로 쓴다. 실험에는 새 코드가 필요 없다.
(바퀴별 상한 `wheel_limit_scale` 과 정리 도구 `mu_surface.py wheels` 는 jjm 에만 있다 — 기록 정리는 랩 PC 에서, NUC 반영은 PR 병합 뒤)
터미널 두 개: **T1 = 스택**, **T2 = 시험**. 한 번에 한 사람만 로봇을 움직인다.

| 순서 | 무엇 | 알아내는 것 | 시간 |
|---|---|---|---|
| ① | 앞·뒤 램프 (`amrmu`) | 노면 μ̂ (뒤 방향 포함) | 20분 |
| ② | 스텝 (`amrstep`) | 노면 μ̂ (가장 깨끗한 방법) | 15분 |
| ③ | 느린 램프 (`amrramp`) | **바퀴별로 먼저 미끄러지는 순서·토크** | 15분 |
| ④ | 바퀴별 하중 (저울) | **바퀴별 무게 N_i** | 10분 |
| ⑤ | 검증 주행 (선택) | μ̂ 0.4 로 덜 미끄러지는지 | 15분 |

## 0. 시작 전 (T1)
```bash
pgrep -af "ros2|ak40|cartographer|rplidar|realsense" | grep -v pgrep     # 아무것도 안 나와야 한다
amrcan                                                                   # NUC 재부팅 뒤라면 (sudo 비밀번호는 직접)
```
- 수동 정지 2명, 배터리 셀당 3.5 V 이상, 바퀴 먼지 닦기, 바닥에 출발 표시
- 공간: ① 앞뒤로 1.5 m, ②·③ 앞·옆으로 2.5 m
- **①~③ 은 같은 자리 바닥에서** (바퀴별 비교가 바닥 차이에 묻히지 않게)

## ① 앞·뒤 램프 시험 (필수)
T1:
```bash
amrall
```
→ 로그에 측위 **"맞음"** 이 나올 때까지 기다린다 (약 15~20 s).

T2:
```bash
amrmu --dirs fwd,back --repeat 5 --csv ~/capston/capston/analysis/torque_mpc_real/2026-11-18_mu/lab_floor_fb
```
- 판마다 Enter. 끝나면 로봇을 출발 표시로 되돌린다
- `★ 슬립 확인 안 됨 — 버림` 이 나온 판은 개수에 안 든다 → 방향마다 **확인된 판이 5개** 가 될 때까지 다시 (예: 뒤만):
```bash
amrmu --dirs back --repeat 3 --csv ~/capston/capston/analysis/torque_mpc_real/2026-11-18_mu/lab_floor_fb2
```
- 끝까지 안 미끄러지면 (`μ̂ ≥ … (하한)`) `--max 0.45` 를 붙여 다시

끝나면 **T1 에서 Ctrl+C** (②·③ 은 스택을 스스로 띄운다).

## ② 스텝 시험 (필수)
T1 (amrall 이 꺼진 상태에서):
```bash
amrstep dir:=fwd tau:=0.3 distance:=1.5 bag:=step_fwd
```
- "맞음" 뒤 **`a` = 한 판**. 1.5 m 가면 스스로 멈춘다 (정지 거리 0.3~0.5 m 더). 로봇을 되돌리고 다시 `a` — **3판**
- `s` 또는 스페이스 = 즉시 정지. 끝나면 Ctrl+C, 그다음 왼쪽 **3판**:
```bash
amrstep dir:=left tau:=0.3 distance:=1.5 bag:=step_left
```
- CSV 는 자동으로 `analysis/torque_mpc_real/2026-11-18_step/`

## ③ 느린 램프 — 바퀴별 슬립 시작 (필수)
스텝은 네 바퀴가 한꺼번에 미끄러져서 바퀴별 차이가 안 보인다. 토크를 천천히 올려 **어느 바퀴가 먼저, 몇 N·m 에서** 헛도는지 본다.
T1:
```bash
amrramp dir:=fwd rate:=0.4 tau_max:=0.5 distance:=1.5 bag:=ramp_fwd
```
- "맞음" 뒤 `a` = 한 판 (토크가 0 → 0.5 N·m 로 1.25 s 동안 오른다, 1.7 m 에서 끊음). **5판**
- 끝나면 Ctrl+C, 왼쪽 **3판**:
```bash
amrramp dir:=left rate:=0.4 tau_max:=0.5 distance:=1.5 bag:=ramp_left
```
- CSV 는 자동으로 `analysis/torque_mpc_real/2026-11-18_ramp/` (바퀴별 속도·모터 보고 토크가 들어 있다)

## ④ 바퀴별 하중 (전원 없이)
저울 1개를 바퀴 하나 밑에, 나머지 세 바퀴는 **저울 윗면과 정확히 같은 높이의 받침** 위에. 두 번씩:

| 바퀴 | 1회 [g] | 2회 [g] |
|---|---|---|
| FL (앞 왼쪽) | | |
| FR (앞 오른쪽) | | |
| RL (뒤 왼쪽) | | |
| RR (뒤 오른쪽) | | |
| 합 (≈ 4560) | | |

- 배터리·케이블 위치는 주행할 때와 똑같이
- **받침 높이가 1 mm 만 달라도 대각 두 바퀴(FL+RR ↔ FR+RL)로 무게가 쏠린다** — 바퀴 네 개 차체는 그렇다.
  11-17 기록에서 FL·RR(대각)이 먼저 미끄러졌으니 이 쏠림일 수 있다. 받침을 바꿔 가며 값이 크게 바뀌면 ③ 결과를 주로 쓴다

## ⑤ 검증 주행 (선택)
T1:
```bash
amrall use_mpc:=true shape:=circles8 width:=2.0 yaw_mode:=tangent speed:=0.4 limit_mode:=friction mu_hat:=0.4 bag:=c_mu04
```
- "맞음" 뒤 `a` = 무장, `s`/스페이스 = 정지. **3판** (판마다 Ctrl+C → 같은 명령 다시)
- 네 바퀴 같은 상한(0.101 N·m)이다. 바퀴별 상한은 NUC 에 코드가 들어간 뒤 `wheel_limit_scale:=…` 로 비교한다
- NUC 팀 체크아웃은 아직 옛 차체 치수(0.1897/0.207)다 (PR #4 병합 전). 비교는 같은 조건인 기존 판과 하므로 그대로 둔다

## 끝나면
- T1·T2 모두 Ctrl+C, 0 의 `pgrep` 으로 남은 프로세스 확인
- 하중 표 값을 채팅으로 알려 주기 → 기록 가져오기·정리는 Claude 가 한다:
  - μ̂: `python3 tools/mu_surface.py summarize --surface lab_floor --summary …_mu/*_summary.csv --write`
  - 바퀴별 배율: `python3 tools/mu_surface.py wheels --loads FL,FR,RL,RR --ramp-node …_ramp/ramp_*.csv --mu <μ̂>`
