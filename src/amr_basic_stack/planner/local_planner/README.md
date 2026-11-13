# Local Planner

- `DRA_MPPI`: 메카넘 AMR용 dynamic risk-aware MPPI reference generator.
  Halton spline으로 400개의 20-step 후보를 만들고, 명시적인 full-stop 후보와
  Monte Carlo joint collision probability를 포함해 최적열을 계산한다.

MPC에는 `/dra_mppi/reference_trajectory`의
`amr_interfaces/PlanarReferenceArray`로 다음 `(20,6)` 전체 최적열을 전달한다.

```text
X*[t] = [x, y, yaw, vx, vy, yaw_rate]
```

현재 MC 설정은 timestep당 2048개이며 모든 rollout이 같은 sample을 공유한다.
세부 구조와 초기 정지 상태에서의 trajectory 생성 과정은
[`DRA_MPPI/README.md`](DRA_MPPI/README.md)를 참고한다.
