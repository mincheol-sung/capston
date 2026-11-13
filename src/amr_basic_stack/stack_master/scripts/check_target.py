#!/usr/bin/env python3
"""C단계 측정 전 — 타깃 후보가 측정에 쓸 만한지 확인.

물체를 놓고 10초 돌려보면 네 가지를 판정한다:
  - 카메라 검출률이 기준(--min-cam-hit) 이상인가
  - 라이다 포착률이 기준(--min-lid-hit) 이상인가
  - 클래스가 한 가지로 안정적인가 (프레임마다 바뀌면 측정 중 검출이 끊긴다)
  - 라이다 클러스터 폭 변동이 기준(--max-width-cv) 이내인가 (경계 불안정 물체 제거)

라이다 포착률을 판정에 넣은 이유:
  폭 변동(CV)만 보면 "360 프레임 중 7 프레임만 잡혔는데 그 7개의 폭이 일정" 인
  경우도 ok=1 이 된다. 표본이 적을수록 CV 는 오히려 작게 나오므로, 포착률 조건이
  없으면 CV 조건은 나쁜 타깃을 통과시키는 방향으로 작동한다.

사용:
    python3 check_target.py --distance 2.0 --duration 10 --label backpack \
        --class-name backpack
    python3 check_target.py                 # 가장 가까운 검출 기준

결과는 매번 check_target_results.csv 에 한 줄씩 append 된다(거리별 곡선용).
"""
import argparse, csv, math, os, sys, time
from collections import Counter
import statistics as st

try:
    import rclpy
    from rclpy.node import Node
    from amr_interfaces.msg import DetectedObjectArray
except ModuleNotFoundError as e:
    missing = e.name or str(e)
    print(f'필요한 ROS2 패키지를 찾지 못했습니다: {missing}', file=sys.stderr)
    print('먼저 아래 명령으로 워크스페이스를 source 하세요:', file=sys.stderr)
    print('  source ~/2026amr_ws/install/setup.bash', file=sys.stderr)
    sys.exit(1)


HERE = os.path.dirname(os.path.abspath(__file__))

# measure_c.py 와 같은 실험 식별자. 결과 파일을 실험 단위로 분리한다.
DEFAULT_EXP = 'c2'


def results_path(exp):
    return os.path.join(HERE, f'check_target_results_{exp}.csv')
# 고정 컬럼. 검출이 없는 실행도 빈 칸으로 기록해야 P_det(d) 곡선에 구멍이 안 생긴다.
# ok 는 임계값에 따라 뜻이 달라지므로 판정에 쓴 임계값 4개를 같이 남긴다.
# 남기지 않으면 --min-lid-hit 0.5 로 돌린 행과 0.9 로 돌린 행이 CSV 에서 구분되지 않는다.
FIELDS = ['ts', 'exp', 'label', 'class_name', 'distance', 'bearing_deg', 'roi', 'duration',
          'cam_frames', 'cam_hit_pct', 'lid_frames', 'lid_hit_pct', 'n_cam', 'n_lid',
          'conf_mean', 'conf_min', 'conf_sd', 'top_class', 'top_class_pct',
          'cam_r_mean', 'cam_r_sd', 'lid_w_mean', 'lid_w_sd', 'lid_w_cv',
          'lid_r_mean', 'lid_r_sd',
          'min_cam_hit', 'min_lid_hit', 'min_class_stability', 'max_width_cv',
          'ok']


def stat(v, f, nd):
    """표본이 모자라면 빈 칸. stdev 는 2개 이상일 때만."""
    if not v or (f is st.stdev and len(v) < 2):
        return None
    return round(f(v), nd)


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


def write_row(row, exp):
    path = csv_path_for(results_path(exp), FIELDS)
    new = not os.path.exists(path) or os.path.getsize(path) == 0
    with open(path, 'a', newline='') as f:
        w = csv.DictWriter(f, fieldnames=FIELDS)
        if new:
            w.writeheader()
        w.writerow(row)
    return path


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--exp', default=DEFAULT_EXP,
                   help='실험 식별자. 결과 파일이 실험 단위로 분리된다 '
                        f'(check_target_results_<exp>.csv). 기본 {DEFAULT_EXP}')
    p.add_argument('--duration', type=float, default=10.0)
    p.add_argument('--distance', type=float, default=None,
                   help='주면 그 위치 근처만 본다(m). 없으면 가장 가까운 검출')
    p.add_argument('--bearing', type=float, default=0.0, help='deg, 좌가 +')
    p.add_argument('--roi', type=float, default=0.8)
    p.add_argument('--label', default='target', help='CSV 에 남길 조건 이름')
    p.add_argument('--class-name', dest='cls_name', default=None,
                   help='이 클래스만 본다(예: person). --distance 없이 타깃 위치를 '
                        '찾을 때, 낮은 conf 임계값에서 섞이는 오검출을 걸러낸다')
    # 판정 임계값 — 기본값은 기존 하드코딩 값과 동일(0.9 / 0.9 / 0.15).
    p.add_argument('--min-cam-hit', type=float, default=0.9,
                   help='카메라 검출률 하한 (0~1)')
    p.add_argument('--min-lid-hit', type=float, default=0.9,
                   help='라이다 포착률 하한 (0~1)')
    p.add_argument('--min-class-stability', type=float, default=0.9,
                   help='최빈 클래스 비율 하한 (0~1)')
    p.add_argument('--max-width-cv', type=float, default=0.15,
                   help='라이다 클러스터 폭 변동계수 상한')
    a = p.parse_args()

    nom = None
    if a.distance is not None:
        th = math.radians(a.bearing)
        nom = (a.distance * math.cos(th), a.distance * math.sin(th))

    rclpy.init(); n = Node('check_target')
    cls = Counter(); cam_tot = cam_hit = 0
    confs = []; cam_r = []
    lid_tot = lid_hit = 0; widths = []; lid_r = []
    last_cam = [None]          # --distance 없을 때 라이다 선택 기준

    def near(o):
        if nom is None:
            return True
        return math.hypot(o.position.x - nom[0], o.position.y - nom[1]) <= a.roi

    def on_cam(m):
        nonlocal cam_tot, cam_hit
        cam_tot += 1
        objs = [o for o in m.objects if near(o)
                and (a.cls_name is None or o.class_name == a.cls_name)]
        if not objs:
            return
        cam_hit += 1
        o = min(objs, key=lambda o: o.distance) if nom is None else \
            min(objs, key=lambda o: math.hypot(o.position.x-nom[0], o.position.y-nom[1]))
        cls[o.class_name] += 1; confs.append(o.confidence); cam_r.append(o.distance)
        last_cam[0] = (o.position.x, o.position.y)

    def on_lid(m):
        nonlocal lid_tot, lid_hit
        # 기준점: --distance 가 있으면 공칭 위치, 없으면 직전 카메라 검출 위치.
        # (여기서는 bias 가 아니라 안정성만 보므로 카메라 기준 선택이 무방)
        ref = nom if nom is not None else last_cam[0]
        if ref is None:
            # 기준점이 아직 없는 프레임은 "라이다가 놓쳤다"가 아니라 "판정 불가"다.
            # 분모에 넣으면 첫 카메라 검출 전 구간만큼 lid_hit_pct 가 깎여서,
            # 포착률을 ok 판정에 쓰는 순간 멀쩡한 타깃이 탈락한다.
            return
        lid_tot += 1
        objs = [o for o in m.objects
                if math.hypot(o.position.x-ref[0], o.position.y-ref[1]) <= a.roi]
        if not objs:
            return
        lid_hit += 1
        o = min(objs, key=lambda o: math.hypot(o.position.x-ref[0], o.position.y-ref[1]))
        widths.append(o.bbox.size_x); lid_r.append(o.distance)

    n.create_subscription(DetectedObjectArray, '/detections', on_cam, 50)
    n.create_subscription(DetectedObjectArray, '/lidar_clusters', on_lid, 50)

    where = f'{a.distance}m/{a.bearing}° 근처' if nom else '가장 가까운 검출'
    cls_txt = f', class={a.cls_name}' if a.cls_name else ''
    print(f'[{a.exp}] {a.duration:.0f}초 확인 중... ({where}{cls_txt})')
    t0 = time.time()
    while time.time() - t0 < a.duration:
        rclpy.spin_once(n, timeout_sec=0.1)

    cam_rate = cam_hit / max(1, cam_tot)
    lid_rate = lid_hit / max(1, lid_tot)
    cls_rate = (cls.most_common(1)[0][1] / sum(cls.values())) if cls else 0.0
    w_cv = (st.stdev(widths) / st.mean(widths)
            if len(widths) > 1 and st.mean(widths) > 0 else None)

    print()
    print(f'  카메라 검출률 {100*cam_rate:5.1f}%   라이다 포착률 {100*lid_rate:5.1f}%'
          f'   ({cam_tot} / {lid_tot} 프레임)')
    if cls:
        print('  클래스 분포: ', end='')
        for k, v in cls.most_common():
            print(f'{k} {100*v/sum(cls.values()):.0f}%  ', end='')
        print()
        print(f'  confidence  평균 {st.mean(confs):.2f}  최소 {min(confs):.2f}')
        print(f'  카메라 거리 평균 {st.mean(cam_r):.3f} m'
              + (f'  σ={st.stdev(cam_r):.4f}' if len(cam_r) > 1 else ''))
    else:
        print('  ✗ 카메라 검출 없음')
    if widths:
        print(f'  라이다 폭   평균 {st.mean(widths):.3f} m'
              + (f'  σ={st.stdev(widths):.4f}' if len(widths) > 1 else '')
              + (f'  CV={100*w_cv:.1f}%' if w_cv is not None else ''))
        print(f'  라이다 거리 평균 {st.mean(lid_r):.3f} m'
              + (f'  σ={st.stdev(lid_r):.4f}' if len(lid_r) > 1 else ''))
    else:
        print('  ✗ 라이다 클러스터 없음')

    print()
    ok = True
    if cam_rate < a.min_cam_hit:
        print(f'  ⚠ 카메라 검출률 {100*a.min_cam_hit:.0f}% 미만 — 조명/각도/거리 조정')
        ok = False
    if lid_rate < a.min_lid_hit:
        print(f'  ⚠ 라이다 포착률 {100*a.min_lid_hit:.0f}% 미만 — 표본이 적어 σ_L 을 믿기 어려움')
        ok = False
    if cls and cls_rate < a.min_class_stability:
        print('  ⚠ 클래스가 흔들림 — 측정 중 검출이 끊길 수 있음'); ok = False
    if w_cv is not None and w_cv > a.max_width_cv:
        print(f'  ⚠ 클러스터 폭 변동 {100*a.max_width_cv:.0f}% 초과 — 경계가 불안정한 물체')
        ok = False
    if ok and cls and widths:
        print('  ✓ 측정에 적합')

    top = cls.most_common(1)[0] if cls else None
    path = write_row({
        'ts': time.strftime('%Y-%m-%d %H:%M:%S'), 'exp': a.exp,
        'label': a.label, 'class_name': a.cls_name,
        'distance': a.distance, 'bearing_deg': a.bearing,
        'roi': a.roi, 'duration': a.duration,
        'cam_frames': cam_tot, 'cam_hit_pct': round(100*cam_rate, 1),
        'lid_frames': lid_tot, 'lid_hit_pct': round(100*lid_rate, 1),
        'n_cam': len(cam_r), 'n_lid': len(lid_r),
        'conf_mean': stat(confs, st.mean, 4), 'conf_min': round(min(confs), 4) if confs else None,
        'conf_sd': stat(confs, st.stdev, 4),
        'top_class': top[0] if top else None,
        'top_class_pct': round(100*top[1]/sum(cls.values()), 1) if top else None,
        'cam_r_mean': stat(cam_r, st.mean, 4), 'cam_r_sd': stat(cam_r, st.stdev, 4),
        'lid_w_mean': stat(widths, st.mean, 4), 'lid_w_sd': stat(widths, st.stdev, 4),
        'lid_w_cv': round(w_cv, 4) if w_cv is not None else None,
        'lid_r_mean': stat(lid_r, st.mean, 4), 'lid_r_sd': stat(lid_r, st.stdev, 4),
        'min_cam_hit': a.min_cam_hit, 'min_lid_hit': a.min_lid_hit,
        'min_class_stability': a.min_class_stability, 'max_width_cv': a.max_width_cv,
        'ok': int(bool(ok and cls and widths)),
    }, a.exp)
    print(f'\n  기록: {path}')
    n.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    main()
