#!/usr/bin/env python3
"""MultiObjectTracker — 프레임 하나를 받아 트랙 집합을 갱신한다. ROS 무관.

한 프레임의 순서:

    1. 모든 트랙을 이번 프레임 시각으로 predict
    2. 활성 트랙을 먼저 배정하고 최근 LOST 트랙을 보수적으로 재연결
    3. 짝지어진 트랙: update (+ 클래스 latch, 운동상태 재판정)
    4. 못 짝지은 트랙: mark_missed  (LOST / DELETED)
    5. 못 짝지은 검출: 새 TENTATIVE 트랙 생성
    6. DELETED 정리

입력 위치·공분산은 이미 추적 프레임(odom)으로 변환된 것이어야 한다. 변환은
ROS 노드의 일이다 — 여기는 TF 를 모른다.

odom 을 쓰는 이유는 정적 물체 때문이다. base_link 에서 보면 로봇이 다가갈 때
정지한 물체가 3.0 -> 2.8 -> 2.6 m 로 움직여 전부 DYNAMIC 이 된다. map 은
그 문제가 없지만 AMCL 재정합 때 이산 점프가 생기는데, 0.2m 점프가 12Hz 두
프레임 사이에 들어오면 겉보기 속도 2.4 m/s 라 이번에는 정지 물체가 전부
DYNAMIC 으로 뒤집힌다. odom 은 연속이고 drift 만 남는다.
"""
import numpy as np

from object_tracker.core import association
from object_tracker.core.config import TrackerConfig
from object_tracker.core.track import (CONFIRMED, DELETED, LOST, TENTATIVE,
                                       Track)


class MultiObjectTracker:

    def __init__(self, cfg=None, nis=None, mode_nis=None, sojourn=None):
        """진단 누적기 세 개를 선택적으로 받는다. 전부 None 이면 진단을 끈다.

        nis       NisAccumulator      fused innovation. 일관성 확인용
        mode_nis  ModeNisAccumulator  mode-conditioned. sigma_a / sigma_a_ct
        sojourn   SojournAccumulator  mean_sojourn_cv / _ct

        실주행에서는 셋 다 필요 없고 파라미터를 실측할 때만 켠다. 켜도 비용은
        부동소수 덧셈 몇 번이다.
        """
        self.cfg = cfg or TrackerConfig()
        self.tracks = []
        self._next_id = 0
        self._last_stamp = None
        self.rejected_timestamps = 0
        self.last_superseded = []
        self.last_measurement_matches = {}
        self.nis = nis
        self.mode_nis = mode_nis
        self.sojourn = sojourn

    @staticmethod
    def _is_person_detection(detection):
        return (detection is not None
                and str(getattr(detection, 'class_name', '')).strip().casefold()
                == 'person')

    @staticmethod
    def _mutual_clear_best(costs, ratio):
        """Keep only mutually best pairs separated clearly from runner-up."""
        costs = np.asarray(costs, dtype=float)
        keep = np.zeros(costs.shape, dtype=bool)
        if not costs.size:
            return keep

        def best_and_clear(values):
            finite = np.flatnonzero(np.isfinite(values))
            if not finite.size:
                return None, False
            order = finite[np.argsort(values[finite])]
            if order.size == 1:
                return int(order[0]), True
            best, second = float(values[order[0]]), float(values[order[1]])
            return int(order[0]), best < ratio * second

        row_choice = [best_and_clear(row) for row in costs]
        column_choice = [best_and_clear(column) for column in costs.T]
        for row, (column, row_clear) in enumerate(row_choice):
            if column is None or not row_clear:
                continue
            chosen_row, column_clear = column_choice[column]
            if column_clear and chosen_row == row:
                keep[row, column] = True
        return keep

    def _lost_costs(self, chi2, lost_indices, measurement_indices,
                    measurements, detections, stamp):
        """Normal LOST matching plus conservative, wider person recovery."""
        costs = np.full((len(lost_indices), len(measurement_indices)), np.inf)
        person_costs = np.full_like(costs, np.inf)
        cfg = self.cfg
        for row, track_index in enumerate(lost_indices):
            track = self.tracks[track_index]
            for column, measurement_index in enumerate(measurement_indices):
                value = chi2[track_index, measurement_index]
                if not np.isfinite(value):
                    continue
                detection = detections[measurement_index]
                person_pair = (track.class_name == 'person'
                               and self._is_person_detection(detection))
                if not person_pair:
                    if value <= cfg.gate_chi2:
                        costs[row, column] = value
                    continue
                age = stamp - track.last_update
                distance = float(np.linalg.norm(
                    np.asarray(measurements[measurement_index])
                    - track.estimator.position))
                if (age <= cfg.person_reid_max_age
                        and distance <= cfg.person_reid_max_distance
                        and value <= cfg.person_reid_gate_chi2):
                    person_costs[row, column] = value

        clear = self._mutual_clear_best(
            person_costs, cfg.person_reid_ambiguity_ratio)
        costs[clear] = person_costs[clear]
        return costs

    def _associate(self, chi2, measurements, detections, stamp, rejected):
        """Give visible tracks priority, then recover unambiguous LOST tracks."""
        confirmed = [i for i, track in enumerate(self.tracks)
                     if track.lifecycle == CONFIRMED]
        tentative = [i for i, track in enumerate(self.tracks)
                     if track.lifecycle == TENTATIVE]
        lost = [i for i, track in enumerate(self.tracks)
                if track.lifecycle == LOST]
        matched = {}
        used_measurements = set()

        # A tentative fragment must not steal an observation from an already
        # confirmed identity. Each stage only sees measurements left by the
        # more trusted stage before it.
        for track_indices in (confirmed, tentative):
            remaining = [j for j in range(len(measurements))
                         if j not in used_measurements]
            if not track_indices or not remaining:
                continue
            stage_detections = [detections[j] for j in remaining]
            stage, _, _ = association.assign_observations(
                chi2[np.ix_(track_indices, remaining)],
                self.cfg.gate_chi2, stage_detections)
            stage_matches = {
                track_indices[row]: remaining[column]
                for row, column in stage.items()
            }
            matched.update(stage_matches)
            used_measurements.update(stage_matches.values())

        remaining = [j for j in range(len(measurements))
                     if j not in used_measurements]
        if lost and remaining:
            costs = self._lost_costs(
                chi2, lost, remaining, measurements, detections, stamp)
            second, _, _ = association.assign(
                costs, self.cfg.person_reid_gate_chi2)
            recovered = {lost[row]: remaining[column]
                         for row, column in second.items()}
            matched.update(recovered)
            used_measurements.update(recovered.values())

        unmatched_tracks = [i for i in range(len(self.tracks))
                            if i not in matched]
        unmatched_measurements = [j for j in range(len(measurements))
                                  if j not in used_measurements]
        if rejected is not None and unmatched_tracks and unmatched_measurements:
            association.assign(
                chi2[np.ix_(unmatched_tracks, unmatched_measurements)],
                self.cfg.gate_chi2, rejected)
        return matched, unmatched_tracks, unmatched_measurements

    def _supersede_person_duplicates(self, stamp, newly_confirmed_ids):
        """Remove an old LOST owner only when one new person is unambiguous."""
        self.last_superseded = []
        if not newly_confirmed_ids:
            return
        new_tracks = [track for track in self.tracks
                      if track.id in newly_confirmed_ids
                      and track.lifecycle == CONFIRMED
                      and track.class_name == 'person']
        old_tracks = [track for track in self.tracks
                      if track.lifecycle == LOST and track.class_name == 'person'
                      and stamp - track.last_update <= self.cfg.person_reid_max_age]
        costs = np.full((len(old_tracks), len(new_tracks)), np.inf)
        for row, old in enumerate(old_tracks):
            for column, new in enumerate(new_tracks):
                # They must not have been separately observed: the old owner
                # was already missing before this replacement track was born.
                if old.last_update >= new.created - 1e-9:
                    continue
                delta = new.estimator.position - old.estimator.position
                if np.linalg.norm(delta) > self.cfg.person_reid_max_distance:
                    continue
                covariance = old.estimator.P_pp + new.estimator.P_pp
                try:
                    value = float(delta @ np.linalg.solve(covariance, delta))
                except np.linalg.LinAlgError:
                    continue
                if value <= self.cfg.person_reid_gate_chi2:
                    costs[row, column] = value
        clear = self._mutual_clear_best(
            costs, self.cfg.person_reid_ambiguity_ratio)
        for row, column in zip(*np.nonzero(clear)):
            old, new = old_tracks[row], new_tracks[column]
            old.superseded_by = new.id
            old.lifecycle = DELETED
            self.last_superseded.append((old.id, new.id))

    def step(self, stamp, measurements, covariances, detections=None):
        """한 프레임 처리. 반환은 발행 대상 트랙 리스트.

        measurements : [(x, y)]        추적 프레임 좌표
        covariances  : [2x2 ndarray]   같은 프레임의 R
        detections   : [DetectedObject] 또는 None (클래스 latch 용, 순서 일치)
        """
        # Only this call's consumed measurements. Rejected/repeated epochs and
        # coasting tracks must never re-emit a previous observation's beams.
        self.last_measurement_matches = {}
        if detections is None:
            detections = [None] * len(measurements)

        if not np.isfinite(stamp):
            raise ValueError('measurement timestamp must be finite')
        if self._last_stamp is not None and stamp <= self._last_stamp:
            self.rejected_timestamps += 1
            return [t for t in self.tracks if t.publishable]
        self._last_stamp = stamp
        self.last_superseded = []
        # Expired tracks cannot be resurrected by a measurement after a long
        # input outage. End diagnostics before pruning them.
        for trk in self.tracks:
            if stamp - trk.last_update > self.cfg.max_coast:
                trk.lifecycle = DELETED
                if self.sojourn is not None:
                    self.sojourn.end_track(trk.id)
        self.tracks = [t for t in self.tracks if t.lifecycle != DELETED]

        # dt 는 sojourn 추정이 쓴다 — TPM 이 expm(G dt) 라 간격이 필요하다.
        elapsed = [trk.predict(stamp) for trk in self.tracks]

        chi2 = association.cost_matrix(self.tracks, measurements, covariances, detections)
        # 기각된 짝을 세지 않으면 표본이 오른쪽에서 잘린 채로 남아 파라미터가
        # 계통적으로 작게 나온다 (core/nis.py). 둘 중 하나라도 켜져 있으면 센다.
        collecting = self.nis is not None or self.mode_nis is not None
        rejected = [] if collecting else None
        matched, unmatched_t, unmatched_m = self._associate(
            chi2, measurements, detections, stamp, rejected)
        if self.nis is not None:
            self.nis.add_rejected(len(rejected))
        if self.mode_nis is not None:
            self.mode_nis.add_rejected(len(rejected))

        newly_confirmed_ids = set()
        for i, j in matched.items():
            trk = self.tracks[i]
            previous_lifecycle = trk.lifecycle
            previous_probabilities = trk.mode_probabilities
            trk.update(measurements[j], covariances[j], stamp, detections[j])
            self.last_measurement_matches[trk.id] = j
            if previous_lifecycle == TENTATIVE and trk.lifecycle == CONFIRMED:
                newly_confirmed_ids.add(trk.id)
            if self.nis is not None:
                self.nis.add_accepted(trk.nis, hits=trk.hits)
            if self.mode_nis is not None:
                self.mode_nis.add_accepted(trk.mode_nis,
                                           trk.mode_probabilities,
                                           hits=trk.hits)
            if self.sojourn is not None:
                self.sojourn.add(trk.id, trk.mode_probabilities,
                                 previous_probabilities, elapsed[i])
        for i in unmatched_t:
            self.tracks[i].mark_missed(stamp)
        for j in unmatched_m:
            self.last_measurement_matches[self._next_id] = j
            self.tracks.append(Track(self._next_id, measurements[j],
                                     covariances[j], stamp, self.cfg,
                                     detections[j]))
            self._next_id += 1

        self._supersede_person_duplicates(stamp, newly_confirmed_ids)

        if self.sojourn is not None:
            for t in self.tracks:
                if t.lifecycle == DELETED:
                    self.sojourn.end_track(t.id)
        self.tracks = [t for t in self.tracks if t.lifecycle != DELETED]
        return [t for t in self.tracks if t.publishable]

    @property
    def next_id(self):
        return self._next_id


def _selfcheck():
    """두 물체 시나리오 — 하나는 정지, 하나는 교차해 지나간다.

    교차 시점에 ID 가 바뀌지 않는지가 관전 포인트다. 위치만으로는 두 트랙이
    겹치지만 각자의 예측(속도)이 달라서 Mahalanobis 가 갈라 준다.
    """
    from object_tracker.core.track import LIFECYCLE_NAME, MOTION_NAME

    cfg = TrackerConfig()
    trk = MultiObjectTracker(cfg)
    R = np.eye(2) * 0.004 ** 2
    dt = 1.0 / 12.0
    rng = np.random.default_rng(7)

    static_p = np.array([3.0, 0.0])
    moving_p = np.array([3.0, -2.0])
    moving_v = np.array([0.0, 0.8])

    print('  정지 물체 (3.0, 0.0),  이동 물체 (3.0, -2.0) 에서 +y 0.8 m/s')
    print('  두 물체는 t = 2.5s 에 (3.0, 0.0) 에서 교차한다.')
    print()
    t = 0.0
    for k in range(60):
        t += dt
        moving_p = moving_p + moving_v * dt
        zs = [static_p + rng.normal(0, 0.004, 2),
              moving_p + rng.normal(0, 0.004, 2)]
        rng.shuffle(zs)             # 검출 순서는 프레임마다 뒤바뀔 수 있다
        out = trk.step(t, zs, [R, R])
        if k in (2, 5, 20, 29, 30, 31, 45, 59):
            print(f'  t={t:5.2f}s  트랙 {len(trk.tracks)}개 발행 {len(out)}개')
            for o in out:
                print(f'      #{o.id} {LIFECYCLE_NAME[o.lifecycle]:10s}'
                      f'{MOTION_NAME[o.motion]:8s} '
                      f'p=({o.kf.x[0]:5.2f},{o.kf.x[1]:+5.2f}) '
                      f'|v|={o.kf.speed:.3f}  chi2_v={o.chi2_v:8.2f}')
    print()
    ids = sorted(t_.id for t_ in trk.tracks)
    print(f'  최종 트랙 ID {ids}, 총 생성 {trk.next_id}개')
    print('  -> 교차 구간에서 ID 가 새로 안 생겼으면 association 이 버틴 것이다.')


if __name__ == '__main__':
    _selfcheck()
