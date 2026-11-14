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
| [`2026-11-14_bench`](2026-11-14_bench/RESULTS_KO.md) | 0~3 (+ NUC 시뮬 확인) | NUC 빌드 시뮬 비교, DDS static peers 문제, CAN 끊김 시험, 모터 4개 status·damping·마찰 계단(오름·내림), 3단계 장착 부호 |
