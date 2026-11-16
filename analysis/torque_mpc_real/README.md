# 토크 MPC 실차 데이터

`amr/notes_amr/real_robot_checklist.md` 를 진행하며 얻은 실차 데이터를 이 폴더 한 곳에 모은다.

## 규칙

- 하위 폴더 하나 = 하루의 한 묶음. 이름은 `YYYY-MM-DD_무엇` (예: `2026-11-14_bench`, `2026-11-20_floor`)
- 폴더마다 `RESULTS_KO.md` — 무엇을 어떤 명령으로 돌렸고, 숫자가 얼마였고, 무엇을 판단했는지
- 리포에 두는 것: 요약 md, 작은 CSV·JSON (벤치 CSV 처럼 폴더당 수 MB 까지)
- **rosbag 원본(`.db3`·`.mcap`)은 리포에 넣지 않는다.** `.gitignore` 가 `bags/`·`*.db3`·`*.mcap` 을 막는다(2026-09 기준 23 GB).
  원본은 작업공간의 `bags/<같은 폴더 이름>/` 에 두고, `RESULTS_KO.md` 에 그 경로·용량·토픽 목록만 적는다

## 목록

| 폴더 | 체크리스트 단계 | 내용 |
|---|---|---|
| [`2026-11-14_bench`](2026-11-14_bench/RESULTS_KO.md) | 0~4 (+ NUC 시뮬 확인) | NUC 빌드 시뮬 비교, DDS static peers 문제, CAN 끊김 시험, 모터 4개 status·damping·마찰 계단(오름·내림), 3단계 CAN 노드(부호·100 Hz·늦은 응답 고침·bus-off), 4단계 일부(전면 라이다 미연결) |
| [`2026-11-14_floor`](2026-11-14_floor/RESULTS_KO.md) | 5 (첫 바닥 8자) | smc 첫 판: 무장 전 측위 어긋남 찾아 `/initialpose`, B·제한 해제·0.3 m/s 에서 5.5 s 만에 30 cm 정지(옆걸음 슬립 발산). MPC·슬립 CSV, bag 은 `bags/2026-11-14_floor/run1` |
| [`2026-11-15_lidar`](2026-11-15_lidar/) | 5-1 | 손으로 밀기 표시(`marks.csv`) — 결과는 아래 `_basic` 문서 |
| [`2026-11-15_basic`](2026-11-15_basic/RESULTS_KO.md) | 5-1·5-2 | 라이다 오도메트리, 앞·뒤 × 토크, 줄자 확인, 빠를 때 SLAM 오류와 bag 재생 원인 찾기(translation_weight) |
| [`2026-11-16_rot`](2026-11-16_rot/RESULTS_KO.md) | 5-2 (4-4 이어서) | 8자 범위 회전(ω 1.2~1.6)에서 추정기 거짓 속도 0.06~0.10 m/s, 0.1~0.3 s |
