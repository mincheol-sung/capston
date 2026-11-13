#!/usr/bin/env python3
"""트랙 하나 — CV/CT/STATIC IMM + 생명주기 + 확률 기반 운동상태 + 클래스.

두 FSM 은 서로 독립이다.

    생명주기  TENTATIVE -> CONFIRMED -> LOST -> DELETED
              "이 트랙을 믿을 만한가 / 언제 버리는가"

    운동상태  UNKNOWN -> {STATIC <-> DYNAMIC}
              "지금 움직이는가"

ChatGPT 설계 논의에서 제일 중요했던 점 — tracker 를 정적용/동적용 둘로 나누지
않는다. 트랙 하나가 상태만 바꾸므로 사람이 멈췄다 다시 걷거나 세워둔 카트가
출발해도 ID 가 끊기지 않는다.

운동상태는 측정 update 뒤 P(STATIC), P(CV)+P(CT)를 비교하여 정한다.
TENTATIVE는 UNKNOWN이며 확률 문턱, hysteresis, 초 단위 확인 시간을 적용한다.
미검출/장기 입력 공백은 연속 증거를 끊는다. chi2_v는 진단 값으로만 유지한다.
"""
import numpy as np

from object_tracker.core.imm import MODE_COUNT, ImmEstimator
from object_tracker.core.kalman import mahalanobis, velocity_chi2
from object_tracker.core.motion_state import MotionClassifier

# 생명주기
TENTATIVE = 0
CONFIRMED = 1
LOST = 2
DELETED = 3
LIFECYCLE_NAME = {TENTATIVE: 'TENTATIVE', CONFIRMED: 'CONFIRMED',
                  LOST: 'LOST', DELETED: 'DELETED'}

# 운동상태.  TrackedObject.msg 의 MOTION_* 상수와 값이 같아야 한다.
UNKNOWN = 0
STATIC = 1
DYNAMIC = 2
MOTION_NAME = {UNKNOWN: 'UNKNOWN', STATIC: 'STATIC', DYNAMIC: 'DYNAMIC'}

# 클래스가 없는 관측. lidar_cluster_node 의 CLASS_ID_LIDAR 와 같아야 한다.
CLASS_ID_LIDAR = -1
# 카메라가 아직 클래스를 준 적이 없는 트랙의 class_name.
TRACK_UNKNOWN_CLASS_NAME = ''


class Track:

    def __init__(self, track_id, z, R, stamp, cfg, detection=None):
        self.id = track_id
        self.estimator = ImmEstimator(z, R, cfg)
        # Read-compatible alias for existing diagnostics/tools.  It exposes the
        # moment-matched x/P and Kalman-like covariance block properties.
        self.kf = self.estimator
        self.cfg = cfg

        self.created = stamp
        self.stamp = stamp             # 마지막 predict 시각
        self.last_update = stamp       # 마지막 measurement 를 받은 시각

        self.lifecycle = TENTATIVE
        self.hits = 1
        self.misses = 0

        self.motion = UNKNOWN
        self.chi2_v = 0.0
        self.nis = float('nan')        # 마지막 update 의 NIS. 첫 관측은 없다.
        # mode 별 NIS 와 그때의 posterior mode 확률. sigma_a / sigma_a_ct 를
        # 따로 식별하려면 fused NIS 하나로는 부족하다 (core/nis.py).
        self.mode_nis = (float('nan'),) * MODE_COUNT
        self.mode_probabilities = tuple(float(p) for p in self.estimator.mu)
        self.motion_classifier = MotionClassifier(cfg)
        # Set only when a later confirmed person demonstrably replaces this
        # identity. The tracker removes superseded tracks before publication.
        self.superseded_by = None

        self.class_id = CLASS_ID_LIDAR
        self.class_name = TRACK_UNKNOWN_CLASS_NAME
        self.confidence = 0.0
        self.last_semantic_update = None
        self.last_person_update = None
        self.person_hits = 0
        self.last_measurement_source = -1
        self.latch_class(detection)

    # ------------------------------------------------------------------ #

    def latch_class(self, detection):
        """Confirm person evidence repeatedly; LiDAR cannot renew its lifetime."""
        if detection is None:
            return
        self.last_measurement_source = int(getattr(
            detection, 'source', 1 if detection.class_id == CLASS_ID_LIDAR else 0))
        if (self.last_measurement_source not in (0, 2)
                or detection.class_id == CLASS_ID_LIDAR):
            return
        if detection.class_name.strip().lower() == 'person':
            if (self.last_person_update is None or self.stamp - self.last_person_update
                    > self.cfg.person_confirmation_window):
                self.person_hits = 0
            self.person_hits += 1
            self.last_person_update = self.stamp
            # Existing person tracks renew immediately; new identities need
            # repeated camera evidence, not three mixed camera/LiDAR hits.
            if self.class_name != 'person' and self.person_hits < self.cfg.person_confirm_hits:
                return
        else:
            self.person_hits = 0
        self.last_semantic_update = self.stamp
        self.class_id = int(detection.class_id)
        self.class_name = str(detection.class_name).strip().lower()
        self.confidence = float(detection.confidence)

    # ------------------------------------------------------------------ #

    def predict(self, stamp):
        """stamp 까지 시간을 진행시킨다. dt <= 0 이면 아무것도 하지 않는다.

        검출은 ApproximateTimeSynchronizer 를 거쳐 오므로 주기가 고르지 않다.
        고정 dt 를 가정하지 않고 매번 header.stamp 차이를 쓴다.
        """
        dt = stamp - self.stamp
        if dt <= 0.0:
            return 0.0
        remaining = dt
        while remaining > 1e-12:
            step = min(remaining, self.cfg.max_dt)
            self.estimator.predict(step)
            remaining -= step
        self.stamp = stamp
        if (self.class_name == 'person' and self.last_semantic_update is not None
                and stamp - self.last_semantic_update > self.cfg.person_label_timeout):
            self.class_id = CLASS_ID_LIDAR
            self.class_name = TRACK_UNKNOWN_CLASS_NAME
            self.confidence = 0.0
            self.person_hits = 0
        return dt

    def update(self, z, R, stamp, detection=None):
        if stamp < self.stamp or stamp <= self.last_update:
            return
        y, S = self.estimator.update(z, R)
        # NIS = y^T S^-1 y. 필터가 일관적이면 chi^2_2 를 따른다. sigma_a 를
        # 실측으로 뽑는 유일한 통계량이라 여기서 남긴다 (core/nis.py).
        self.nis = mahalanobis(y, S)
        # mode 별로도 남긴다. fused NIS 는 스칼라 하나라 척도 파라미터를
        # 하나밖에 식별하지 못한다 — sigma_a 와 sigma_a_ct 를 분리하려면
        # mode-conditioned NIS 를 mode 확률로 가중해야 한다.
        self.mode_nis = tuple(
            mahalanobis(residual, innovation_covariance)
            for residual, innovation_covariance
            in self.estimator.mode_innovations)
        self.mode_probabilities = tuple(float(p) for p in self.estimator.mu)
        self.last_update = stamp
        self.hits += 1
        self.misses = 0
        if self.lifecycle == LOST:
            self.lifecycle = CONFIRMED
        elif self.lifecycle == TENTATIVE and self.hits >= self.cfg.n_confirm:
            self.lifecycle = CONFIRMED
        self.latch_class(detection)
        self._update_motion()

    def mark_missed(self, stamp):
        """이번 프레임에 짝지어지지 않았다. predict 결과만 남는다."""
        self.misses += 1
        self.motion_classifier.reset_evidence()
        self.motion_classifier.state = UNKNOWN
        if self.lifecycle == TENTATIVE:
            # 확정 전 트랙은 한 번만 놓쳐도 버린다. 잡음에서 태어난 트랙을
            # 오래 끌면 다음 프레임의 진짜 검출을 가로챈다(SORT 와 같은 처리).
            self.lifecycle = DELETED
            return
        if self.lifecycle == CONFIRMED:
            self.lifecycle = LOST
        if stamp - self.last_update > self.cfg.max_coast:
            self.lifecycle = DELETED

    # ------------------------------------------------------------------ #

    def _update_motion(self):
        # Retained only for diagnostics, never used to classify motion.
        self.chi2_v = velocity_chi2(self.kf)
        self.motion = self.motion_classifier.update(
            self.stamp, self.estimator.mu, self.lifecycle == CONFIRMED)

    # ------------------------------------------------------------------ #

    @property
    def age(self):
        return self.stamp - self.created

    @property
    def is_dynamic(self):
        return self.motion == DYNAMIC

    @property
    def radius(self):
        return float(self.cfg.default_obstacle_radius)

    @property
    def publishable(self):
        return self.lifecycle in (CONFIRMED, LOST)

    def __repr__(self):
        x = self.kf.x
        return (f'Track#{self.id} {LIFECYCLE_NAME[self.lifecycle]}/'
                f'{MOTION_NAME[self.motion]} p=({x[0]:.2f},{x[1]:.2f}) '
                f'v=({x[2]:+.2f},{x[3]:+.2f}) |v|={self.kf.speed:.2f} '
                f'chi2_v={self.chi2_v:.2f} hits={self.hits} miss={self.misses} '
                f'class={self.class_name or "-"}')


def _selfcheck():
    from object_tracker.core.config import TrackerConfig

    cfg = TrackerConfig()
    R = np.eye(2) * 0.004 ** 2      # C2 기준 4m 융합 위치 잡음 ~4mm
    dt = 1.0 / 12.0
    rng = np.random.default_rng(1)

    print(f'  cfg: sigma_a={cfg.sigma_a} sigma_v0={cfg.sigma_v0} '
          f'probability enter S={cfg.static_enter_probability:.2f} '
          f'D={cfg.dynamic_enter_probability:.2f}')
    print()

    for label, speed in (('정지 (v=0)', 0.0), ('보행 (v=1.2 m/s)', 1.2)):
        p = np.array([3.0, 0.0])
        t = 0.0
        trk = Track(0, p, R, t, cfg)
        hist = []
        for k in range(60):
            t += dt
            p = p + np.array([speed, 0.0]) * dt
            trk.predict(t)
            trk.update(p + rng.normal(0, 0.004, 2), R, t)
            hist.append((k, trk.motion, trk.chi2_v, trk.kf.speed))
        print(f'=== {label} ===')
        for k, m, c, s in hist:
            if k in (2, 3, 5, 10, 20, 40, 59):
                print(f'  k={k:3d}  {MOTION_NAME[m]:8s} chi2_v={c:10.2f} '
                      f'|v_hat|={s:.3f}')
        print()

    print('=== 생명주기: TENTATIVE 는 한 번 놓치면 삭제 ===')
    trk = Track(1, np.zeros(2), R, 0.0, cfg)
    print(f'  생성 직후      {LIFECYCLE_NAME[trk.lifecycle]} hits={trk.hits}')
    trk.mark_missed(0.1)
    print(f'  1회 미검출 후  {LIFECYCLE_NAME[trk.lifecycle]}')

    print()
    print('=== 생명주기: CONFIRMED -> LOST -> 재검출 -> CONFIRMED ===')
    t = 0.0
    trk = Track(2, np.zeros(2), R, t, cfg)
    for _ in range(cfg.n_confirm - 1):
        t += dt
        trk.predict(t)
        trk.update(np.zeros(2), R, t)
    print(f'  hits={trk.hits}  {LIFECYCLE_NAME[trk.lifecycle]}')
    t += dt
    trk.predict(t)
    trk.mark_missed(t)
    print(f'  1회 미검출     {LIFECYCLE_NAME[trk.lifecycle]}')
    t += dt
    trk.predict(t)
    trk.update(np.zeros(2), R, t)
    print(f'  재검출         {LIFECYCLE_NAME[trk.lifecycle]}')
    t += cfg.max_coast + dt
    trk.predict(t)
    trk.mark_missed(t)
    print(f'  {cfg.max_coast}s 공백    {LIFECYCLE_NAME[trk.lifecycle]}')


if __name__ == '__main__':
    _selfcheck()
