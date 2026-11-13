#!/usr/bin/env python3
"""트래커 설계 파라미터 한 곳.

ROS 노드가 declare_parameter 로 같은 이름을 노출하고 여기 기본값을 그대로
쓴다. 파라미터 이름·기본값의 단일 출처가 이 파일이다.

값의 근거를 세 등급으로 나눠 둔다. 주석의 [실측] / [유도] / [TODO] 표시가
그 등급이다.

  [실측]  C2 실험(2026-09-04)에서 잰 값이거나 그로부터 계산된 값.
  [유도]  통계·기하에서 닫힌 형태로 나오는 값. 실험이 필요 없다.
  [TODO]  아직 근거가 없는 임시값. 실측으로 대체해야 한다.

STATIC/DYNAMIC은 3-mode IMM posterior 확률과 관측 유지 시간으로 판정한다.
process noise와 전환율은 mode likelihood를 지배하므로 함께 보정해야 한다.
최근 bag 기반 초기값의 근거와 한계는 docs/THREE_MODE_IMM.md에 기록한다.

재는 방법은 core/nis.py 에 구현돼 있다 — NIS (normalized innovation squared)
일관성 검정, Bar-Shalom 5.4절. 정답 궤적(ground truth)이 필요 없다.
실행은 tools/nis_fit.py:

    python3 tools/nis_fit.py --sim 1.0        # 도구 자체 검증
    python3 tools/nis_fit.py --bag <bag경로>   # 실측

주의할 것 두 가지 (교재에 그냥 "평균이 2" 라고만 나오는 부분이다):
  - association 게이트가 NIS <= 9.21 인 표본만 남기므로 목표는 2.000 이
    아니라 1.907 이다. 2.000 을 쓰면 4.9% 계통오차가 된다.
  - 게이트 밖으로 밀려난 개수까지 세서 절단 MLE 를 써야 한다. 절단 평균만
    쓰면 sigma_a 가 2배 이상 틀렸을 때 포화해서 못 되돌아온다.
"""
from dataclasses import dataclass
import math

from object_tracker.core.kalman import chi2_2dof


@dataclass
class TrackerConfig:
    # ── 운동 모델 ────────────────────────────────────────────────────── #

    # [TODO] CWNA 가속도 세기 [m/s^1.5]. 단위가 m/s^2 가 아닌 이유는
    # kalman.py 모듈 독스트링에 있다 (Q_vv = sigma_a^2 dt 가 (m/s)^2 여야
    # 하므로 sigma_a^2 는 m^2/s^3). 보행자 기준 통상값 1.0 을 임시로 둔다.
    #
    # 아래 수치는 과거 CV/속도 chi2 판정 실험(12Hz, 4m)의 참고 기록이다.
    # 현재 3-mode posterior 판정의 최소 검출 속도를 뜻하지 않는다.
    # 조작적 정의 셋:
    #   1) predict 한 번이 속도 추정에 넣는 불확실성이 sigma_a sqrt(dt).
    #      sigma_a=1.0 이면 288.7 mm/s.
    #   2) T 초 coast 하면 sigma_v(T) = sqrt(sigma_v0^2 + sigma_a^2 T).
    #      즉 "1초 눈 감았을 때 속도 추정이 벌어지는 폭"이 곧 sigma_a 다.
    #   3) 정상상태 sigma_v 를 거의 선형으로 지배하고, 과거 판정 하한이
    #      그 sqrt(9.21) = 3.035 배다.
    #
    # 그래서 반대 방향 둘을 동시에 지배한다. tools/detection_floor.py 로 잰
    # 하한과, 게이트(chi^2 9.21)가 깨지는 표적 가속도를 이분탐색한 값:
    #
    #     sigma_a   판정 하한 (1~9m)     게이트가 깨지는 a
    #     0.15       73 ~  160 mm/s       0.84 m/s^2
    #     0.3       142 ~  259 mm/s       1.86 m/s^2
    #     0.5              -              3.49 m/s^2
    #     1.0       471 ~  602 mm/s       7.76 m/s^2   <- 현재
    #     3.0      1412 ~ 1488 mm/s      하한이 보행 속도를 넘는다. 못 쓴다.
    #
    # 참고 기동: 90도 선회 1.70, 급정지 2.40, 뛰다 급정지 6.00 m/s^2.
    # 현재는 mode likelihood와 association을 함께 바꾸므로 STATIC 잡음과
    # 전환율을 포함해 재검증해야 한다. 과거 표만으로 판정 성능을 보장하지 않는다.
    sigma_a: float = 1.0

    # CT mode의 Cartesian acceleration 및 turn-rate random-walk 세기. 아직
    # 실측값이 없으므로 bag 기반 NIS/mode-likelihood 검증으로 조정해야 한다.
    sigma_a_ct: float = 0.7
    sigma_omega_cv: float = 0.15
    sigma_omega_ct: float = 0.5
    sigma_omega0: float = 1.0

    # CV mode가 mixing으로 받은 omega를 0으로 되돌리는 시정수 [s].
    cv_omega_decay_time: float = 0.25

    # 연속시간 Markov chain의 평균 mode 체류시간 [s]. 매 실제 dt마다
    # Pi(dt)=expm(G*dt)를 계산하므로 불규칙한 검출 주기에도 의미가 유지된다.
    mean_sojourn_cv: float = 3.0
    mean_sojourn_ct: float = 1.0
    mean_sojourn_static: float = 3.0
    static_to_cv: float = 0.9
    cv_to_static: float = 0.5
    ct_to_static: float = 0.2
    sigma_position_static: float = 0.04
    static_velocity_decay_time: float = 0.1
    initial_probability_cv: float = 0.4
    initial_probability_static: float = 0.4
    minimum_mode_probability: float = 1.0e-4

    # [유도] 새 트랙의 속도 불확실성 [m/s]. 추적 대상이 낼 수 있는 속도의
    # 표준편차. 보행 1.2 m/s, 뛰면 3 m/s 이므로 1.5 면 3sigma 가 4.5 m/s 로
    # 충분히 열린다. 작게 잡으면 움직이는 물체가 몇 프레임 뒤처져 붙는다.
    sigma_v0: float = 1.5

    # Maximum integration substep [s]. Longer gaps use multiple substeps;
    # elapsed measurement time must never be silently discarded.
    max_dt: float = 0.5

    # ── association ──────────────────────────────────────────────────── #

    # [유도] chi^2_{2, 0.99} = 9.21034. camera_lidar_fusion 의 게이트와 같은 값·
    # 같은 논리다. 자유도 2 는 -2 ln(1-p) 로 닫힌 형태다.
    gate_chi2: float = chi2_2dof(0.99)

    # [실측] 측정 공분산이 비어 있을 때(전부 0) 쓸 대체 sigma [m].
    # fusion output은 항상 채워 보내므로 정상 경로에서는 쓰이지 않는다.
    # 원본 /detections 를 직접 물렸을 때 R 이 특이행렬이 되는 것만 막는다.
    fallback_sigma: float = 0.10

    # Engineering defaults, not calibrated probabilities. LiDAR observations
    # never renew semantic evidence. Geometry remains tracked after expiry.
    person_confirm_hits: int = 3
    person_confirmation_window: float = 1.0
    # A LiDAR-only continuation must not keep a stale semantic identity for
    # several seconds.  One second still bridges short camera dropouts while
    # preventing an old person label from migrating through anonymous clusters.
    person_label_timeout: float = 1.0
    class_conflict_confidence: float = 0.6

    # A LOST person gets one conservative re-identification chance before a
    # new identity is created. These are engineering defaults; the hard
    # distance and ambiguity checks prevent a wide statistical gate from
    # joining two nearby people blindly.
    person_reid_max_age: float = 0.50
    person_reid_gate_chi2: float = 16.0
    person_reid_max_distance: float = 0.60
    person_reid_ambiguity_ratio: float = 0.70

    # metric footprint 측정이 아직 없으므로 prediction에 넣을 원형 반경 fallback.
    default_obstacle_radius: float = 0.25

    # ── 생명주기 ─────────────────────────────────────────────────────── #

    # [TODO] CONFIRMED 로 올리는 데 필요한 누적 검출 수.
    n_confirm: int = 3

    # [TODO] 마지막 검출 이후 이만큼 지나면 삭제 [s]. 프레임 수가 아니라 시간인
    # 이유: 검출 주기가 ApproximateTimeSynchronizer 때문에 고르지 않다.
    max_coast: float = 1.0

    # ── STATIC / DYNAMIC 판별 ────────────────────────────────────────── #

    # Provisional IMM probability thresholds, not calibrated class accuracy.
    static_enter_probability: float = 0.85
    static_exit_probability: float = 0.65
    dynamic_enter_probability: float = 0.80
    dynamic_exit_probability: float = 0.60

    # Continuous measured evidence in seconds, reset on misses/long input gaps.
    static_confirm_time: float = 0.5
    dynamic_confirm_time: float = 0.3
    motion_evidence_max_gap: float = 0.25

    # ------------------------------------------------------------------ #

    @classmethod
    def field_defaults(cls):
        """{이름: 기본값}. ROS 노드가 declare_parameter 에 그대로 쓴다."""
        return {f.name: getattr(cls, f.name, f.default)
                for f in cls.__dataclass_fields__.values()}

    def validate(self):
        for name in self.field_defaults():
            value = getattr(self, name)
            if isinstance(value, (int, float)) and not math.isfinite(value):
                raise ValueError(f'{name} must be finite')
        if self.max_dt <= 0.0 or self.max_coast <= 0.0:
            raise ValueError('max_dt and max_coast must be positive')
        if self.fallback_sigma <= 0 or self.gate_chi2 <= 0 or self.n_confirm < 1:
            raise ValueError('fallback_sigma, gate_chi2 and n_confirm must be positive')
        if self.person_confirm_hits < 1:
            raise ValueError('person_confirm_hits must be positive')
        if self.person_confirmation_window <= 0 or self.person_label_timeout <= 0:
            raise ValueError('person evidence durations must be positive')
        if not 0 <= self.class_conflict_confidence <= 1:
            raise ValueError('class_conflict_confidence must be in [0,1]')
        if min(self.person_reid_max_age, self.person_reid_gate_chi2,
               self.person_reid_max_distance) <= 0:
            raise ValueError('person re-identification scales must be positive')
        if self.person_reid_gate_chi2 < self.gate_chi2:
            raise ValueError('person_reid_gate_chi2 must be >= gate_chi2')
        if self.person_reid_max_age > self.max_coast:
            raise ValueError('person_reid_max_age must not exceed max_coast')
        if not 0 < self.person_reid_ambiguity_ratio < 1:
            raise ValueError('person_reid_ambiguity_ratio must be in (0,1)')
        if self.sigma_a < 0.0 or self.sigma_a_ct < 0.0:
            raise ValueError('process-noise strengths must be non-negative')
        if self.sigma_omega_cv < 0.0 or self.sigma_omega_ct < 0.0:
            raise ValueError('turn-rate noise strengths must be non-negative')
        if self.sigma_v0 <= 0.0 or self.sigma_omega0 <= 0.0:
            raise ValueError('initial standard deviations must be positive')
        if self.cv_omega_decay_time < 0.0:
            raise ValueError('cv_omega_decay_time must be non-negative')
        if self.mean_sojourn_cv <= 0.0 or self.mean_sojourn_ct <= 0.0:
            raise ValueError('mean sojourn times must be positive')
        if (self.initial_probability_cv <= 0 or self.initial_probability_static <= 0
                or self.initial_probability_cv + self.initial_probability_static >= 1):
            raise ValueError('initial CV/STATIC probabilities must leave positive CT mass')
        if not 0.0 <= self.minimum_mode_probability < 1.0 / 3.0:
            raise ValueError('minimum_mode_probability must be in [0,1/3)')
        if self.default_obstacle_radius <= 0.0:
            raise ValueError('default_obstacle_radius must be positive')
        if min(self.mean_sojourn_static, self.static_velocity_decay_time,
               self.sigma_position_static, self.motion_evidence_max_gap) <= 0:
            raise ValueError('STATIC model and evidence gap scales must be positive')
        for name in ('static_to_cv', 'cv_to_static', 'ct_to_static'):
            if not 0 <= getattr(self, name) <= 1:
                raise ValueError(f'{name} must be in [0,1]')
        for prefix in ('static', 'dynamic'):
            enter = getattr(self, prefix + '_enter_probability')
            leave = getattr(self, prefix + '_exit_probability')
            if not 0.5 < leave < enter < 1.0:
                raise ValueError(f'{prefix} requires 0.5 < exit < enter < 1')
            if getattr(self, prefix + '_confirm_time') < 0:
                raise ValueError('confirmation times must be non-negative')


def _selfcheck():
    cfg = TrackerConfig()
    print('=== TrackerConfig 기본값 ===')
    for k, v in TrackerConfig.field_defaults().items():
        print(f'  {k:20s} {v}')
    print()
    print(f'  gate_chi2          = chi2_2dof(0.99) = {cfg.gate_chi2:.5f}')
    cfg.validate()


if __name__ == '__main__':
    _selfcheck()
