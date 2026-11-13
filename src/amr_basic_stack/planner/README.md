# Planner

Planning packages are split by scope:

- `global_planner`: map-based global path planning with Theta*.
  `/traversable_map` (`amr_interfaces/TraversableMap`, 0 통과 가능 / 100 통과
  불가 / -1 미확인) 이 신선하면 저장 PGM 대신 현재 계획용 지도로 쓴다. 미확인은 통과 허용이
  기본이며 `theta_star.yaml` 의 `traversable_unknown_is_obstacle` 로 바꾼다.
  계약은 `perception/traversability/README.md` 참고.
- `local_planner/DRA_MPPI`: Halton-spline/Monte-Carlo 기반 dynamic risk-aware
  MPPI. 20-step의 `[x,y,yaw,vx,vy,yaw_rate]` 전체 최적열을
  `/dra_mppi/reference_trajectory`로 하위 MPC에 전달한다.
