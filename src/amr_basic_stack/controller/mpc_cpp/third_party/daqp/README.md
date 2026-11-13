# DAQP (vendored)

- 출처: https://github.com/darnstrom/daqp  태그 v0.10.3 (커밋 29a8987d8b9a596c1ab6acf84c17d38b068b73bc, 2026-10-02)
- 라이선스: MIT (`LICENSE`)
- 넣은 것: `src/*.c`, `include/*.h` 만. 수정 없음. 빌드는 상위 `CMakeLists.txt` 의 `daqp_vendored` 타깃.
- 선정 근거: 2026-10-22 100 Hz 폐루프 QP 1600 개(변수 80, 제약 320) 벤치마크에서 qpOASES 대비 최악 시간
  0.25 ms vs 0.87 ms (hotstart) / 1.37 ms (cold), Python qpactive 해와 최대 8e-14.
- 갱신: 새 태그의 `src/`, `include/` 를 통째로 덮어쓰고 `wheel_mpc_replay` 일치 검증을 다시 돌린다.
