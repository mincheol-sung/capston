#!/usr/bin/env python3
"""C단계 측정 — 카메라/라이다 각각의 σ(d), GT 대비 bias, 센서간 bias 를 실측.

여기서 재는 σ 의 의미(중요):
  σ_C(d) 는 D455 depth 소자의 intrinsic noise 가 **아니다**. 아래 파이프라인
  전체의 프레임간 변동, 즉 object-level measurement uncertainty 다.
      Camera: RGB-D -> YOLO detection -> depth ROI -> 3D representative position
              -> TF -> object measurement
      LiDAR : LaserScan -> ABD clustering -> cluster representative position
              -> TF -> object measurement
  따라서 YOLO 임계값(yolo_conf), depth_roi_ratio, ABD λ 를 바꾸면 σ 도 바뀐다.
  거리 기반 measurement covariance 모델의 입력으로 쓰기에 맞는 양은 이쪽이다.

대표점(representative position)의 의미:
  두 센서 모두 물체의 **보이는 표면점**만 관측한다. r_C, r_L 은 물체의 기하학적
  중심까지의 거리가 아니라 visible-point representative position 까지의 거리다.
  bias 에는 "서로 다른 표면을 본다"는 성분이 구조적으로 남는다(센서 잡음 아님).

핵심 설계: 두 센서의 타깃을 **서로를 참조하지 않고 독립적으로** 선택한다.
  선택 기준 = 사용자가 아는 공칭 위치(--distance, --bearing)로부터의 거리.
  상대 센서 위치 기준으로 고르면(= 최근접 매칭) "가장 덜 어긋난 것"을 고르는 셈이라
  측정하려는 bias 가 측정 방법에 의해 지워진다(순환논리).
  따라서 d_CL 은 필터 조건이 아니라 **측정 결과**로 출력한다(→ gate 설계 참고값).
  σ 도 각 센서의 전체 표본에서 계산해 gate 에 의한 꼬리 잘림이 없다.
  pair() 결과는 오직 센서간 비교(bias_CL, d_CL)에만 쓰고 σ 계산에는 쓰지 않는다.

nominal 과 GT 의 분리(중요):
  --distance     = association/ROI 탐색용 **공칭** 거리. 실제 거리가 아니다.
  --gt-distance  = 줄자로 실측한 Ground Truth 거리. 반올림하지 말고 그대로 넣는다.
  목표가 2.0 m 여도 실측이 2.034 m 면 그 run 은 σ_C(2.034) 라는 데이터점이다.
  두 값을 CSV 에 nominal_d / gt_d 로 따로 저장한다.

전제(중요): 타깃 주변 반경 --roi 안에 다른 물체가 없어야 한다.
  그래야 공칭 위치 기준 선택이 곧 올바른 association 이 된다.
  → association 을 알고리즘이 아니라 실험 설계로 보장하는 것.

전제2: /detections 와 /lidar_clusters 는 서로 독립인 관측이어야 한다.
  rgbd_detection_node 는 라이다를 읽지 않으므로(거리별 융합 LidarRefiner 제거됨)
  이 조건은 코드 구조로 보장된다. 두 토픽을 합치는 노드를 나중에 넣을 때는
  그 출력이 아니라 여기 두 원본 토픽을 계속 구독해야 한다.

사용법 (스택 실행 중):
    python3 measure_c.py --distance 2.0 --gt-distance 2.034 --duration 30 \
        --label backpack --class-name backpack
    python3 measure_c.py --distance 2.0 --gt-distance 2.034 --bearing -30 \
        --duration 30 --label backpack_R30 --class-name backpack
"""
import argparse
import csv
import math
import os
import statistics as st
import time

import rclpy
from rclpy.node import Node
from amr_interfaces.msg import DetectedObjectArray

HERE = os.path.dirname(os.path.abspath(__file__))

# 실험 식별자. 결과 파일을 실험 단위로 **분리**한다.
#   c1 = 2026-08-20~21 측정 (center_patch 대표점, GT 없음) -> measure_c_results.csv, _v2
#   c2 = 이번 실험 (roi_median_3d 대표점, GT 기록)         -> measure_c_results_c2.csv
# 대표점 정의가 바뀌면 σ 는 다른 측정량이 된다. 같은 파일에 이어붙이면 회귀에
# 섞여 들어가고, 섞인 뒤에는 되돌릴 수 없다. 파일 분리가 가장 싼 방어선이다.
DEFAULT_EXP = 'c2'


def results_path(exp):
    return os.path.join(HERE, f'measure_c_results_{exp}.csv')

# 고정 컬럼. bias 계열은 짝이 하나도 안 잡히면 빠지는데, 그때마다 헤더를
# 다시 만들면 이어붙인 행들이 서로 어긋난다(헤더는 첫 행에서 한 번만 쓰므로).
FIELDS = ['ts', 'exp', 'label', 'class_name', 'nominal_d', 'gt_d', 'gt_ref',
          'bearing_deg', 'roi',
          'n_cam', 'n_lid', 'cam_hit_pct', 'lid_hit_pct',
          'cam_r_mean', 'cam_sigma_r', 'cam_bias_gt',
          'lid_r_mean', 'lid_sigma_r', 'lid_bias_gt',
          'bias_CL_r', 'bias_CL_th',
          'cam_th_mean', 'cam_sigma_th', 'lid_th_mean', 'lid_sigma_th',
          'n_pair', 'd_CL_mean', 'd_CL_p95', 'd_CL_max', 'dt_mean', 'dt_max',
          'quality_ok', 'quality_note', 'note', 'raw_file']

# raw 표본 — 분포/아웃라이어/자기상관/부트스트랩 분석용. 한 run 을 그대로 복원한다.
RAW_FIELDS = ['run_ts', 'exp', 'label', 'sensor', 'nominal_d', 'gt_d', 'bearing_deg',
              'timestamp', 'x', 'y', 'z', 'r', 'theta',
              'class_name', 'confidence', 'source']


def polar(o):
    return math.hypot(o.position.x, o.position.y), math.atan2(o.position.y, o.position.x)


def mean_of(v):
    return st.mean(v) if v else None


def sd_of(v):
    return st.stdev(v) if len(v) > 1 else None


def pct(v, q):
    """단순 백분위(선형보간 없음). 표본이 적을 때 과신하지 않도록 보수적으로."""
    if not v:
        return None
    s = sorted(v)
    return s[min(len(s) - 1, int(math.ceil(q * len(s))) - 1)]


def csv_path_for(path, fields):
    """헤더가 다른 기존 파일에는 이어붙이지 않는다.

    DictWriter 는 헤더를 첫 행에서 한 번만 쓰므로, 컬럼이 바뀐 뒤 같은 파일에
    append 하면 옛 헤더 아래에 새 순서의 값이 들어가 열이 조용히 어긋난다.
    스키마가 다르면 _v3, _v4 ... 로 파일을 나눈다(기존 데이터는 건드리지 않음).
    """
    base, ext = os.path.splitext(path)
    for i in range(0, 8):
        cand = path if i == 0 else f'{base}_v{i + 2}{ext}'
        if not os.path.exists(cand) or os.path.getsize(cand) == 0:
            return cand
        with open(cand, newline='') as f:
            head = next(csv.reader(f), [])
        if head == fields:
            return cand
    raise RuntimeError(f'{path}: 스키마가 다른 파일이 너무 많습니다')


class Collector(Node):
    def __init__(self, a):
        super().__init__('measure_c')
        self.a = a
        th = math.radians(a.bearing)
        self.nom = (a.distance * math.cos(th), a.distance * math.sin(th))

        self.cam = []      # 표본 dict — 독립 선택
        self.lid = []
        self.cam_frames = self.lid_frames = 0
        self.cam_hit = self.lid_hit = 0

        self.create_subscription(DetectedObjectArray, '/lidar_clusters', self._on_lidar, 50)
        self.create_subscription(DetectedObjectArray, '/detections', self._on_cam, 50)

    @staticmethod
    def _t(m):
        return m.header.stamp.sec + m.header.stamp.nanosec * 1e-9

    def _pick(self, objs):
        """공칭 위치에 가장 가까운 객체. ROI 밖이면 None."""
        if not objs:
            return None
        best = min(objs, key=lambda o: math.hypot(o.position.x - self.nom[0],
                                                  o.position.y - self.nom[1]))
        if math.hypot(best.position.x - self.nom[0],
                      best.position.y - self.nom[1]) > self.a.roi:
            return None
        return best

    @staticmethod
    def _sample(t, o):
        r, th = polar(o)
        return {'t': t, 'x': o.position.x, 'y': o.position.y, 'z': o.position.z,
                'r': r, 'th': th, 'class_name': o.class_name,
                'confidence': float(o.confidence), 'source': int(o.source)}

    def _on_cam(self, msg):
        self.cam_frames += 1
        # 클래스 필터는 카메라에만 적용한다. 라이다 클러스터에는 클래스가 없으므로
        # 라이다는 공칭 위치 기준 선택을 그대로 쓴다(센서간 독립성 유지).
        objs = [o for o in msg.objects
                if self.a.cls_name is None or o.class_name == self.a.cls_name]
        o = self._pick(objs)
        if o is not None:
            self.cam_hit += 1
            self.cam.append(self._sample(self._t(msg), o))

    def _on_lidar(self, msg):
        self.lid_frames += 1
        o = self._pick(list(msg.objects))
        if o is not None:
            self.lid_hit += 1
            self.lid.append(self._sample(self._t(msg), o))


def pair(cam, lid, max_dt):
    """one-to-one 최근접 시각 짝짓기. 센서간 비교(bias_CL, d_CL) 전용.

    옛 구현은 카메라 표본마다 최근접 라이다를 골라서, 라이다 프레임이 더 느리면
    같은 라이다 표본이 여러 카메라 표본에 재사용됐다. 그러면 n_pair 는 커지는데
    독립 정보량은 그대로라 bias 의 신뢰구간이 실제보다 좁게 보인다.
    여기서는 |Δt| 가 작은 후보부터 확정하고, 한 번 쓴 표본은 양쪽 다 재사용하지 않는다.
    """
    cam = sorted(cam, key=lambda s: s['t'])
    lid = sorted(lid, key=lambda s: s['t'])
    cand = []
    for i, c in enumerate(cam):
        for j, l in enumerate(lid):
            dt = abs(l['t'] - c['t'])
            if dt <= max_dt:
                cand.append((dt, i, j))
    cand.sort()

    used_c, used_l, out = set(), set(), []
    for dt, i, j in cand:
        if i in used_c or j in used_l:
            continue
        used_c.add(i); used_l.add(j)
        out.append((cam[i], lid[j], dt))
    out.sort(key=lambda p: p[0]['t'])
    return out


def write_raw(a, c, run_ts):
    """한 run 의 모든 표본을 그대로 보존. 요약만 남기면 되돌릴 수 없다."""
    name = f'measure_c_raw_{a.exp}_{time.strftime("%Y%m%d_%H%M%S")}.csv'
    path = os.path.join(HERE, name)
    with open(path, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=RAW_FIELDS)
        w.writeheader()
        for sensor, rows in (('camera', c.cam), ('lidar', c.lid)):
            for s in rows:
                w.writerow({
                    'run_ts': run_ts, 'exp': a.exp, 'label': a.label, 'sensor': sensor,
                    'nominal_d': a.distance, 'gt_d': a.gt_distance,
                    'bearing_deg': a.bearing,
                    'timestamp': f"{s['t']:.6f}",
                    'x': round(s['x'], 6), 'y': round(s['y'], 6), 'z': round(s['z'], 6),
                    'r': round(s['r'], 6), 'theta': round(s['th'], 8),
                    'class_name': s['class_name'],
                    'confidence': round(s['confidence'], 4),
                    'source': s['source'],
                })
    return name


def summarize(a, c, run_ts):
    nc, nl = len(c.cam), len(c.lid)
    cam_rate = c.cam_hit / max(1, c.cam_frames)
    lid_rate = c.lid_hit / max(1, c.lid_frames)

    print()
    print('=' * 70)
    gt_txt = f'{a.gt_distance} m' if a.gt_distance is not None else '(미입력)'
    print(f'  [{a.exp}] {a.label}   공칭 {a.distance} m / {a.bearing}°   '
          f'GT {gt_txt}   ROI {a.roi} m')
    if a.cls_name:
        print(f'  카메라 클래스 필터: {a.cls_name}')
    if a.note:
        print(f'  파이프라인: {a.note}')
    print('=' * 70)
    print(f'  타깃 포착률   카메라 {100*cam_rate:5.1f}% ({nc}샘플)   '
          f'라이다 {100*lid_rate:5.1f}% ({nl}샘플)')

    # 표본이 적어도 행을 버리지 않는다. 낮은 검출률 자체가 reliability 데이터다.
    # 대신 quality_ok=0 으로 표시해서 모델 피팅에서 걸러낼 수 있게 한다.
    if nc < a.min_samples or nl < a.min_samples:
        print(f'  ⚠ 샘플 부족(<{a.min_samples}). quality_ok=0 으로 기록만 합니다.')

    rC = [s['r'] for s in c.cam]; thC = [s['th'] for s in c.cam]
    rL = [s['r'] for s in c.lid]; thL = [s['th'] for s in c.lid]

    cam_r_mean, lid_r_mean = mean_of(rC), mean_of(rL)
    o = {'ts': run_ts, 'exp': a.exp,
         'label': a.label, 'class_name': a.cls_name,
         'nominal_d': a.distance,
         # 반올림하지 않는다. 입력한 실측값 그대로가 이 run 의 거리다.
         'gt_d': a.gt_distance, 'gt_ref': a.gt_ref,
         'bearing_deg': a.bearing, 'roi': a.roi, 'note': a.note,
         'n_cam': nc, 'n_lid': nl,
         'cam_hit_pct': round(100*cam_rate, 1),
         'lid_hit_pct': round(100*lid_rate, 1),
         'cam_r_mean': cam_r_mean, 'cam_sigma_r': sd_of(rC),
         'cam_th_mean': mean_of(thC), 'cam_sigma_th': sd_of(thC),
         'lid_r_mean': lid_r_mean, 'lid_sigma_r': sd_of(rL),
         'lid_th_mean': mean_of(thL), 'lid_sigma_th': sd_of(thL)}

    # ── GT 대비 bias (센서별 절대 정확도) ──────────────────────────────
    # 각 센서 표본 전체의 평균에서 GT 를 뺀다. pair 결과를 쓰지 않는다.
    if a.gt_distance is not None:
        if cam_r_mean is not None:
            o['cam_bias_gt'] = cam_r_mean - a.gt_distance
        if lid_r_mean is not None:
            o['lid_bias_gt'] = lid_r_mean - a.gt_distance

    if nc or nl:
        print()
        print(f'  {"":10}{"평균 r(m)":>11}{"σ_r(m)":>10}{"σ_θ(rad)":>12}{"σ_θ·r 횡(m)":>13}')
        for tag, rm, sr, sth in (('카메라', o['cam_r_mean'], o['cam_sigma_r'], o['cam_sigma_th']),
                                 ('라이다', o['lid_r_mean'], o['lid_sigma_r'], o['lid_sigma_th'])):
            if rm is None:
                print(f'  {tag:10}{"—":>11}')
                continue
            s1 = f'{sr:10.4f}' if sr is not None else f'{"—":>10}'
            s2 = f'{sth:12.5f}' if sth is not None else f'{"—":>12}'
            s3 = f'{sth*rm:13.4f}' if sth is not None else f'{"—":>13}'
            print(f'  {tag:10}{rm:11.3f}{s1}{s2}{s3}')

    # ── bias 3종 ─────────────────────────────────────────────────────
    pr = pair(c.cam, c.lid, a.max_dt)
    if pr:
        br = st.mean([p[0]['r'] - p[1]['r'] for p in pr])
        bt = st.mean([p[0]['th'] - p[1]['th'] for p in pr])
        dcl = [math.hypot(p[0]['x'] - p[1]['x'], p[0]['y'] - p[1]['y']) for p in pr]
        dts = [p[2] for p in pr]
        o.update({'n_pair': len(pr), 'bias_CL_r': br, 'bias_CL_th': bt,
                  'd_CL_mean': st.mean(dcl), 'd_CL_p95': pct(dcl, 0.95),
                  'd_CL_max': max(dcl),
                  'dt_mean': st.mean(dts), 'dt_max': max(dts)})

    print()
    print('  편향 (bias)')
    if 'cam_bias_gt' in o:
        print(f'    Camera GT bias    : {o["cam_bias_gt"]:+.4f} m'
              f'   (r̄_C {cam_r_mean:.3f} − d_GT {a.gt_distance})')
    else:
        print('    Camera GT bias    : —   (--gt-distance 미입력 또는 표본 없음)')
    if 'lid_bias_gt' in o:
        print(f'    LiDAR GT bias     : {o["lid_bias_gt"]:+.4f} m'
              f'   (r̄_L {lid_r_mean:.3f} − d_GT {a.gt_distance})')
    else:
        print('    LiDAR GT bias     : —   (--gt-distance 미입력 또는 표본 없음)')
    if pr:
        print(f'    Camera-LiDAR bias : {o["bias_CL_r"]:+.4f} m   '
              f'(θ {math.degrees(o["bias_CL_th"]):+.2f}°, '
              f'횡 {o["bias_CL_th"]*(cam_r_mean or 0):+.4f} m, {len(pr)}쌍)')
        # 센서 오차로 30cm 는 나올 수 없다. 이 크기면 association 이 깨진 것이다.
        # 흔한 형태: 공칭이 실제 타깃보다 뒷벽에 더 가까워서 라이다가 벽을 뽑는다.
        # 이때 σ_lid 는 오히려 아주 작게 나오므로(벽은 안 움직임) σ 만 보면 못 잡는다.
        if abs(o['bias_CL_r']) > 0.3:
            print()
            print(f'    ✗✗ Camera-LiDAR bias = {o["bias_CL_r"]:+.3f} m 는 '
                  '센서 오차로 설명되지 않는 크기입니다.')
            print('       두 센서가 서로 다른 물체를 보고 있을 가능성이 큽니다')
            print('       (라이다가 타깃 뒤 벽·기둥을 잡는 경우가 가장 흔함).')
            print('       타깃 뒤를 1 m 이상 비우고, --distance 를 실측값에 맞추세요.')
        print()
        print(f'  d_CL distribution/gate 설계 참고값   '
              f'평균 {o["d_CL_mean"]:.3f} m,  p95 {o["d_CL_p95"]:.3f} m,  '
              f'최대 {o["d_CL_max"]:.3f} m')
        print('    (최대값 하나로 gate 를 정하지 말 것 — 표본 하나에 좌우된다.'
              ' 거리별 분포를 모아 percentile/공분산 기준으로 설계)')
        print(f'  동기   Δt 평균 {o["dt_mean"]*1000:.1f} ms, '
              f'최대 {o["dt_max"]*1000:.1f} ms  ({len(pr)}쌍, one-to-one)')
    else:
        print('    Camera-LiDAR bias : —   (짝지어진 표본 없음)')

    # ── quality flag ────────────────────────────────────────────────
    bad = []
    if cam_rate < a.min_cam_hit:
        bad.append(f'cam_hit<{100*a.min_cam_hit:.0f}%')
    if lid_rate < a.min_lid_hit:
        bad.append(f'lid_hit<{100*a.min_lid_hit:.0f}%')
    if nc < a.min_samples:
        bad.append(f'n_cam<{a.min_samples}')
    if nl < a.min_samples:
        bad.append(f'n_lid<{a.min_samples}')
    o['quality_ok'] = int(not bad)
    o['quality_note'] = ';'.join(bad)
    print()
    print(f'  quality_ok = {o["quality_ok"]}'
          + (f'   ({o["quality_note"]})' if bad else '   (모든 조건 통과)'))

    if o['cam_sigma_r'] and o['lid_sigma_r']:
        print()
        print('  → 이 거리의 역분산 가중치:')
        for nm, sc, sl in (('반경', o['cam_sigma_r'], o['lid_sigma_r']),
                           ('방위', o['cam_sigma_th'], o['lid_sigma_th'])):
            if sc and sl and sc > 0 and sl > 0:
                wc = (1/sc**2) / ((1/sc**2) + (1/sl**2))
                print(f'       {nm}  w_cam={wc:.4f}   w_lidar={1-wc:.4f}')
    return o


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--exp', default=DEFAULT_EXP,
                   help='실험 식별자. 결과 파일이 실험 단위로 분리된다 '
                        f'(measure_c_results_<exp>.csv). 기본 {DEFAULT_EXP}')
    p.add_argument('--distance', type=float, required=True,
                   help='타깃 **공칭** 거리 (m). association/ROI 탐색용이며 GT 가 아니다')
    p.add_argument('--gt-distance', type=float, default=None,
                   help='줄자 등으로 실측한 Ground Truth 거리 (m). 반올림하지 말 것')
    p.add_argument('--gt-ref', choices=['front', 'center'], default='front',
                   help='GT 를 물체의 어디까지 쟀는지. bias 해석이 달라지므로 CSV 에 기록')
    p.add_argument('--bearing', type=float, default=0.0, help='타깃 공칭 방위 (deg, 좌가 +)')
    p.add_argument('--duration', type=float, default=30.0)
    p.add_argument('--label', default='target', help='실험 이름(클래스와 무관)')
    p.add_argument('--class-name', dest='cls_name', default=None,
                   help='/detections 에서 이 클래스만 카메라 후보로 쓴다(예: backpack). '
                        '라이다는 클래스가 없으므로 공칭 위치 기준 그대로')
    p.add_argument('--roi', type=float, default=0.6,
                   help='공칭 위치 기준 탐색 반경(m). 주변에 다른 물체가 없어야 함')
    p.add_argument('--max-dt', type=float, default=0.1,
                   help='센서간 비교(pair) 허용 시간차 (s)')
    # quality_ok 판정 기준. 기준 미달이어도 행은 반드시 기록한다.
    p.add_argument('--min-cam-hit', type=float, default=0.7, help='카메라 포착률 하한 (0~1)')
    p.add_argument('--min-lid-hit', type=float, default=0.7, help='라이다 포착률 하한 (0~1)')
    p.add_argument('--min-samples', type=int, default=10, help='센서별 최소 표본 수')
    p.add_argument('--no-raw', action='store_true', help='raw 표본 CSV 를 남기지 않음')
    # σ 는 파이프라인 설정에 따라 달라진다(yolo_conf, camera_position_method,
    # depth_roi_ratio, ABD λ ...). 그 설정을 CSV 에 안 남기면 나중에 행들을 합칠 때
    # 무엇끼리 비교 가능한지 알 수 없다. measure_both.sh 가 자동으로 채워 넣는다.
    p.add_argument('--note', default='',
                   help='이 run 의 파이프라인 설정 메모(예: cam_pos=roi_median_3d;conf=0.15)')
    a = p.parse_args()

    if a.gt_distance is None:
        print('⚠ --gt-distance 미입력: gt_d / cam_bias_gt / lid_bias_gt 는 빈 칸이 됩니다.')
        print('  공칭 거리(--distance)를 GT 로 대체하지 않습니다(다른 개념이므로).')

    rclpy.init()
    c = Collector(a)
    print(f'{a.duration:.0f}초 수집: {a.label} @ 공칭 {a.distance}m / {a.bearing}°'
          + (f'  (GT {a.gt_distance}m, {a.gt_ref} 기준)' if a.gt_distance is not None else ''))
    t0 = time.time()
    while time.time() - t0 < a.duration:
        rclpy.spin_once(c, timeout_sec=0.1)

    run_ts = time.strftime('%Y-%m-%d %H:%M:%S')
    o = summarize(a, c, run_ts)

    if not a.no_raw and (c.cam or c.lid):
        o['raw_file'] = write_raw(a, c, run_ts)
        print(f'  raw 표본: {os.path.join(HERE, o["raw_file"])}')

    path = csv_path_for(results_path(a.exp), FIELDS)
    new = not os.path.exists(path) or os.path.getsize(path) == 0
    with open(path, 'a', newline='') as f:
        w = csv.DictWriter(f, fieldnames=FIELDS, restval='')
        if new:
            w.writeheader()
        w.writerow(o)
    print(f'  기록: {path}')

    c.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
