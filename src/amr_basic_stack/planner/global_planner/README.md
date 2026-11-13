# Global Planner

이 패키지의 운용 노드 `rviz_theta_star_node`는 저장된 ROS map과 RViz goal을
사용해 Theta* global path를 만들고 `/global_path`로 발행한다.

## 현재 구현

```text
/map + /scan + depth ──→ traversability_node ──→ /traversable_map
                                                      │
                         신선함 ──→ Theta* 기준 지도·남은 경로 검사
                         timeout ─→ 목표 계획은 저장 PGM fallback
                                   주행 중 검사·재계획은 건너뜀
                                                      │
RViz /initialpose + /goal_pose ───────────────────→ Theta* ──→ /global_path
                                                               │
                                                   막히면 현재 위치에서 재계획
```

검사는 waypoint만 보는 것이 아니라 waypoint 사이의 모든 grid cell을 확인한다.
재계획에는 검사와 동일한 신선한 traversable map을 사용하므로, 새 장애물을
발견하고도 원래 경로를 그대로 다시 만드는 문제를 피한다. 성공한 재계획은 빈
path를 먼저 발행하지 않고 기존 path에서 새 path로 바로 교체한다. 재계획할 경로가
없을 때만 빈 `/global_path`를 유지하고 다음 검사 주기에 다시 시도한다.

신선한 `/traversable_map`은 저장 PGM과 OR하지 않고 Theta*의 기준 지도로 사용한다.
따라서 인지가 FREE로 재관측한 셀은 저장 PGM의 오래된 장애물을 해제할 수 있다.
2초 동안 traversable map이 끊기면 새 목표 계획은 저장 PGM으로 fallback하고, 주행
중 경로 검증과 재계획은 신선한 지도가 다시 들어올 때까지 건너뛴다. 저장된 map과
`self.costmap` 자체는 수정하지 않는다.

RViz에서 찍은 start 또는 goal이 현재 planning costmap에서 막힌 경우에는 경로 요청을
바로 거부하지 않는다. 설정된 반경 안의 가장 가까운 collision-free cell로 endpoint를
옮겨 Theta*를 실행하며, goal이 이동된 경우 `/global_path`도 그 안전 셀에서 끝난다.
반경 안에 안전 셀이 없거나 map 밖을 클릭한 경우에는 기존처럼 경로를 만들지 않는다.

### 파라미터

| 파라미터 | 기본값 | 의미 |
|---|---:|---|
| `endpoint_snap_radius` | `0.50` | 막힌 start/goal을 가장 가까운 안전 셀로 이동할 최대 거리 [m] |
| `path_validation_frequency` | `1.0` | 남은 경로 검사 주기 [Hz] |
| `use_traversable_map` | `true` | 신선한 `/traversable_map`을 저장 PGM 대신 기준 지도로 사용 |
| `traversable_map_timeout` | `2.0` | 이 시간 동안 수신이 없으면 저장 PGM으로 fallback [s] |
| `traversable_unknown_is_obstacle` | `false` | traversable map의 미확인 셀을 장애물로 볼지 여부 |

위 값은 `config/theta_star.yaml`에서 관리한다. `map`, `maps_dir`, `use_sim_time`은
실행 환경에 따라 달라지므로 `autonomy.launch.py`가 YAML 값을 덮어쓴다.

```bash
ros2 launch stack_master autonomy.launch.py map:=test_imu_scan
```

`use_traversable_map: false`로 설정하면 traversable map 구독과 검사 타이머를
실행하지 않으며, 정적 map으로 목표 수신 시 한 번만 경로를 생성한다.

별도 설정 파일을 사용할 때는 원본을 수정하지 않고 launch argument로 파일 경로를
지정할 수 있다.

```bash
ros2 launch stack_master autonomy.launch.py map:=test_imu_scan \
  theta_star_config:=/absolute/path/to/theta_star.yaml
```

## 현재 방식의 한계

- 센서에서 현재 보이는 범위와 관측 기억만 알 수 있어 가려진 장애물 판단에는 한계가 있다.
- 움직이는 사람도 잠시 정적 장애물처럼 보이므로 상황에 따라 재계획이 흔들릴 수 있다.
- 1 Hz 검사는 global replan용이다. 즉각 정지와 동적 회피는 local planner 또는 별도
  safety layer가 담당해야 한다.
- localization 또는 traversable map의 좌표 정합이 틀리면 장애물이 map의 잘못된
  위치에 반영된다.
