# AK40-10 실기 토크 시험 결과

모든 추정값은 출력축 단위·명령토크 추종을 가정한 후보값이다. 실제 전류는 측정하지 않았다.
차체 질량·차체 yaw 관성·노면 마찰은 이번 시험으로 측정하지 않았다. 로봇 설정은 변경하지 않았다.

## 비교용 J,C 단순 모델 (T = J·dω/dt + C·ω)

| 바퀴 | 상태 | 응답/기록 | 최대 속도 rad/s | 최대 온도 °C | J kg·m² | C N·m·s/rad |
|---|---|---:|---:|---:|---:|---:|
| FL(1) | completed | 11100/11100 | 9.233 | 28 | 0.000859255 | 0.00706788 |
| FR(2) | aborted | 9615/9615 | 9.900 | 30 | 0.000936736 | 0.00797845 |
| RL(3) | aborted | 8640/8640 | 9.122 | 29 | 0.000919158 | 0.00703888 |
| RR(4) | completed | 11100/11100 | 9.567 | 33 | 0.000865896 | 0.00547595 |

## 최종 피팅 모델: 관성 + 쿨롱 마찰 + 점성 마찰

T = J·dω/dt + τc·sign(ω) + bv·ω + offset. 입력 토크와 각속도에 5 Hz LPF를 적용한 뒤 적분식으로 피팅했다. 잔차비가 작을수록 단순 J,C 모델보다 잘 맞는다.

| 바퀴 | J kg·m² | τc N·m | bv N·m·s/rad | 확장/단순 잔차비 |
|---|---:|---:|---:|---:|
| ID 1 / FL | 0.00086548 | 0.0280067 | 0.00113138 | 0.216 |
| ID 2 / FR | 0.000888916 | 0.0278573 | 0.00235076 | 0.162 |
| ID 3 / RL | 0.00091774 | 0.0238585 | 0.00220756 | 0.215 |
| ID 4 / RR | 0.000875873 | 0.025327 | 0.000800279 | 0.144 |

## 모터 ID별 데이터와 중단 사유

- ID 1 / FL: [/home/junseo/motor_identification_logs/20261111_181835_078402_BENCH](/home/junseo/motor_identification_logs/20261111_181835_078402_BENCH) — completed, 111.04초, 11100행. 정상 완료. disable 송신 오류: ['4: [Errno 105] No buffer space available']
- ID 2 / FR: [/home/junseo/motor_identification_logs/20261111_182144_929734_BENCH](/home/junseo/motor_identification_logs/20261111_182144_929734_BENCH) — aborted, 96.19초, 9615행. Scheduler >30ms late; refusing catch-up burst. disable 송신 오류: []
- ID 3 / RL: [/home/junseo/motor_identification_logs/20261111_182331_861708_BENCH](/home/junseo/motor_identification_logs/20261111_182331_861708_BENCH) — aborted, 86.44초, 8640행. Motor 3 did not settle below 0.15rad/s for 0.2s. Aborting.. disable 송신 오류: []
- ID 4 / RR: [/home/junseo/motor_identification_logs/20261111_183226_841201_BENCH](/home/junseo/motor_identification_logs/20261111_183226_841201_BENCH) — completed, 111.04초, 11100행. 정상 완료. disable 송신 오류: []

## 해석

기본 피팅은 원본을 보존한 5 Hz LPF 신호를 사용한다. raw/LPF 후보를 각 폴더에 따로 저장했다.
단순 모델에서 쿨롱 마찰이 C에 흡수될 수 있으므로 두 모델의 C와 bv는 같은 물리량으로 확정하면 안 된다.
이번 토크 자극은 짧고 속도 보호로 일부 잘리므로 정상상태 속도 전체 범위의 마찰을 확정하지 않는다.
명령토크와 실제 토크 사이 지연·스케일, 센서 잡음, 축 기준, 반복 재현성 검토가 남아 있다.
aborted 시험은 일부 데이터만 사용했으며 완료 시험으로 간주하지 않는다.

## 시험 이력

- /home/junseo/motor_identification_logs/20261111_181835_078402_BENCH: completed, 11100행. 
- /home/junseo/motor_identification_logs/20261111_182034_617387_BENCH: aborted, 96행. Scheduler >30ms late; refusing catch-up burst
- /home/junseo/motor_identification_logs/20261111_182044_417135_BENCH: aborted, 96행. Scheduler >30ms late; refusing catch-up burst
- /home/junseo/motor_identification_logs/20261111_182144_929734_BENCH: aborted, 9615행. Scheduler >30ms late; refusing catch-up burst
- /home/junseo/motor_identification_logs/20261111_182331_861708_BENCH: aborted, 8640행. Motor 3 did not settle below 0.15rad/s for 0.2s. Aborting.
- /home/junseo/motor_identification_logs/20261111_182608_700270_BENCH: aborted, 3900행. Motor 4 did not settle below 0.15rad/s for 0.2s. Aborting.
- /home/junseo/motor_identification_logs/20261111_182907_296517_BENCH: completed, 11100행. 
- /home/junseo/motor_identification_logs/20261111_183226_841201_BENCH: completed, 11100행. 

## 원본과 필터 신호

![모터 ID별 각속도와 토크](BENCH_SIGNALS.png)


## 인수인계와 사용 코드

[새 터미널용 인수인계 문서](HANDOFF_KO.md)에 사용 코드·재현 명령·모터 ID·분석 방법·중단 이력을 정리했다. 현재 코드 사본과 SOURCE_MANIFEST.json도 같은 폴더에 있다. ID 4는 20261111_183226_841201_BENCH 재시험을 반영했다.
