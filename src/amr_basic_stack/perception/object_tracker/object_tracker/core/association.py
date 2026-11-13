#!/usr/bin/env python3
"""Track <-> measurement association. Mahalanobis 게이팅 + Hungarian 배정.

camera_lidar_fusion 의 association 과 같은 구조지만 대상이 다르다.

    camera_lidar_fusion :  카메라 검출  <-> 라이다 클러스터   (같은 시각, 다른 센서)
    여기             :  기존 트랙    <-> 이번 프레임 검출  (다른 시각, 같은 관측열)

문턱을 유클리드 거리가 아니라 chi^2 로 두는 이유도 같다. 트랙마다 P 가 다르고
검출마다 R(d) 가 달라서 "몇 m 이내"라는 문턱은 근거리에서 너무 헐겁고 원거리에서
너무 빡빡하다. innovation covariance S = H P H^T + R 로 정규화하면 두 불확실성이
모두 들어간 하나의 척도가 된다.
"""
import numpy as np
from scipy.optimize import linear_sum_assignment

from object_tracker.core.kalman import mahalanobis


def cost_matrix(tracks, measurements, covariances, detections=None):
    """chi2[i, j] = (z_j - H x_i)^T S_ij^-1 (z_j - H x_i).

    tracks 는 이미 이번 프레임 시각으로 predict 된 상태여야 한다.
    """
    n, m = len(tracks), len(measurements)
    chi2 = np.full((n, m), np.inf)
    for i, trk in enumerate(tracks):
        for j, (z, R) in enumerate(zip(measurements, covariances)):
            detection = detections[j] if detections is not None else None
            if (detection is not None and trk.class_id >= 0
                    and detection.class_id >= 0
                    and trk.class_id != detection.class_id
                    and min(trk.confidence, detection.confidence)
                    >= trk.cfg.class_conflict_confidence):
                continue
            # Track supplies the moment-matched IMM prediction.
            y, S = trk.estimator.innovation(z, R)
            chi2[i, j] = mahalanobis(y, S)
    return chi2


def assign(chi2, gate, rejected=None):
    """Maximize gate-valid matches, then minimize their total chi-square cost.

    Each row has an explicit unmatched option. Forbidden pairs cannot displace
    valid ones. Optional rejection diagnostics use an ungated assignment on
    only the remaining rows/columns; these are candidates, not known true pairs.
    """
    n, m = chi2.shape
    matched = {}
    if n and m:
        valid = np.isfinite(chi2) & (chi2 <= gate)
        unmatched_cost = (min(n, m) + 1) * (gate + 1.0)
        cost = np.full((n, m + n), unmatched_cost)
        cost[:, :m] = np.where(valid, chi2, unmatched_cost * (n + 2))
        rows, cols = linear_sum_assignment(cost)
        for i, j in zip(rows, cols):
            if j < m and valid[i, j]:
                matched[int(i)] = int(j)
    used = set(matched.values())
    unmatched_t = [i for i in range(n) if i not in matched]
    unmatched_m = [j for j in range(m) if j not in used]
    if rejected is not None and unmatched_t and unmatched_m:
        residual = chi2[np.ix_(unmatched_t, unmatched_m)]
        finite = np.isfinite(residual)
        if finite.any():
            big = (float(residual[finite].max()) + gate + 1) * (n + m + 1)
            rows, cols = linear_sum_assignment(np.where(finite, residual, big))
            rejected.extend(float(residual[i, j]) for i, j in zip(rows, cols)
                            if finite[i, j] and residual[i, j] > gate)
    return (matched,
            unmatched_t, unmatched_m)


def assign_observations(chi2, gate, detections, rejected=None):
    """Associate semantic observations first, then LiDAR to remaining tracks.

    A slightly nearer anonymous cluster must not steal a track from a valid
    camera observation in the same synchronized input frame.
    """
    semantic = [j for j, d in enumerate(detections)
                if d is not None and d.class_id >= 0]
    if not semantic:
        return assign(chi2, gate, rejected)
    matched, remaining_rows, _ = assign(chi2[:, semantic], gate)
    matched = {i: semantic[j] for i, j in matched.items()}
    anonymous = [j for j in range(chi2.shape[1]) if j not in semantic]
    second, _, _ = assign(chi2[np.ix_(remaining_rows, anonymous)], gate)
    matched.update({remaining_rows[i]: anonymous[j] for i, j in second.items()})
    remaining_rows = [i for i in range(chi2.shape[0]) if i not in matched]
    used = set(matched.values())
    remaining_cols = [j for j in range(chi2.shape[1]) if j not in used]
    if rejected is not None:
        # Diagnostic candidates only; do not count camera-stage rejects that
        # were subsequently explained by a LiDAR update.
        assign(chi2[np.ix_(remaining_rows, remaining_cols)], gate, rejected)
    return matched, remaining_rows, remaining_cols


def _selfcheck():
    print('=== assign: 게이트 밖 짝은 배정 후 제거된다 ===')
    # 2 트랙 x 2 검출. 전역 최적은 (0,0),(1,1) 이지만 (1,1) 이 게이트 밖.
    chi2 = np.array([[1.0, 50.0],
                     [50.0, 30.0]])
    matched, ut, um = assign(chi2, gate=9.21)
    print(f'  chi2 =\n{chi2}')
    print(f'  matched={matched}  unmatched_tracks={ut}  unmatched_meas={um}')
    assert matched == {0: 0} and ut == [1] and um == [1]

    print()
    print('=== 교차 배정: 탐욕법이면 틀리는 경우 ===')
    # 탐욕법은 최솟값 0.5 를 먼저 잡아 (0,1) 을 고르고 (1,0)=8.0 을 남겨 합 8.5.
    # Hungarian 은 (0,0)+(1,1) = 2.0+1.0 = 3.0 을 고른다.
    chi2 = np.array([[2.0, 0.5],
                     [8.0, 1.0]])
    matched, ut, um = assign(chi2, gate=9.21)
    total = chi2[0, 0] + chi2[1, 1]
    print(f'  matched={matched}  (합 {total:.1f}, 탐욕법이면 8.5)')
    assert matched == {0: 0, 1: 1}

    print()
    print('=== 빈 입력 ===')
    for shape in ((0, 3), (3, 0), (0, 0)):
        matched, ut, um = assign(np.full(shape, np.inf), gate=9.21)
        print(f'  shape={shape}  matched={matched} ut={ut} um={um}')
    print()
    print('전부 통과.')


if __name__ == '__main__':
    _selfcheck()
