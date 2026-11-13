#!/usr/bin/env python3
"""Create tables and exportable figures from saved validation artifacts."""
import argparse
import csv
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np


def read_json(path):
    return json.loads(path.read_text())


def cycle_rows(path):
    return [row for line in path.read_text().splitlines()
            if not (row := json.loads(line)).get('recorder_summary')]


def describe(values):
    a = np.asarray([v for v in values if v is not None], dtype=float)
    if not len(a):
        return None
    return dict(count=len(a), mean=float(np.mean(a)),
                **dict(zip(('p50', 'p95', 'p99', 'max'),
                           np.percentile(a, [50, 95, 99, 100], method='linear').tolist())))


def number(value):
    return '—' if value is None else f'{value:.2f}'


def save_figure(fig, output, name):
    fig.tight_layout()
    for suffix in ('png', 'svg'):
        fig.savefig(output/f'{name}.{suffix}', dpi=160)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input', type=Path, required=True)
    args = parser.parse_args()
    root = args.input
    output = root/'figures'
    output.mkdir(exist_ok=True)
    runs = []
    for path in sorted(root.glob('**/summary.json')):
        if path.parent.name == 'ros_smoke' or 'before_diagnostic_fix' in path.parts:
            continue
        s = read_json(path)
        rows = cycle_rows(path.with_name('cycles.jsonl'))[s['warmup_cycles']:]
        computed = [r for r in rows if r.get('optimizer_ms') is not None]
        s['computed_callback_ms'] = describe(r['callback_ms'] for r in computed)
        s['computed_cycles'] = len(computed)
        s['optimizer_attempt_cycles'] = len(computed)+sum(r['status'] == 'timeout' for r in rows)
        s['by_status'] = {status: describe(r['callback_ms'] for r in rows if r['status'] == status)
                          for status in sorted({r['status'] for r in rows})}
        s['run_id'] = str(path.parent.relative_to(root))
        s['path'] = str(path.relative_to(root))
        s['primary_mc_samples_mean'] = (float(np.mean([r['actual_primary_mc_samples']
            for r in computed])) if computed else None)
        s['candidate_sample_comparisons_mean'] = (float(np.mean([
            r['candidate_sample_comparisons'] for r in computed])) if computed else None)
        s['record_prepare_ms'] = describe(r['record_prepare_ms'] for r in rows)
        s['underresolved_steps'] = sum((r.get('underresolved_steps') or 0) for r in rows)
        s['primary_empty_circles'] = sum((r.get('primary_empty_circles') or 0) for r in rows)
        s['primary_clipped_marginals'] = sum((r.get('primary_clipped_marginals') or 0) for r in rows)
        runs.append((s, rows))
    (root/'timing_summary.json').write_text(json.dumps([s for s, _ in runs], indent=2))
    table = ['# NUC 검증 결과 — 2026-10-22', '',
             '원시 cycle의 NumPy `percentile(method="linear")`로 각 run을 따로 계산했다. '
             '조기 반환·stale·timeout을 전체 callback 통계에서 제외하지 않았다. '
             '계산이 실제 수행된 cycle의 callback 통계와 상태별 통계는 '
             '[timing_summary.json](timing_summary.json)에 별도로 있다.', '',
             '| Run | 전체/MPPI 시도/결과 객체 | mean | p50 | p95 | p99 | max | >200 ms | timeout | skip |',
             '|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|']
    for s, rows in runs:
        stats = s['all_callback_ms']
        table.append('| '+s['run_id']+' | '+f"{s['cycles']}/{s['optimizer_attempt_cycles']}/{s['computed_cycles']}"+' | '+
            ' | '.join(number(stats[k]) for k in ('mean', 'p50', 'p95', 'p99', 'max'))+
            f" | {s['callback_over_200ms_count']}/{s['cycles']} | {s['timeout_count']} | {s['skipped_periods']} |")
    table += ['', '단위 ms. 결과 객체 수에는 비용 평가를 마친 뒤 입력 만료로 폐기한 cycle을 포함한다. '
              'timeout은 MPPI 시도에 포함하지만 결과 객체는 없다. 전체 wall 통계와 timeout 수에 모두 포함하며, 결과 객체 0개를 계산을 하지 않았다는 뜻으로 해석하면 안 된다.', '',
              '| Run | 시작 지연 p99 | 예정 시각→발행 p99 | 발행 간격 p99/max | 수신 간격 p99 | 상태 수 |',
              '|---|---:|---:|---:|---:|---|']
    for s, _ in runs:
        interval = s['publish_interval_ms'] or {}
        table.append(f"| {s['run_id']} | {number((s['start_delay_ms'] or {}).get('p99'))} | "
                     f"{number((s['scheduled_response_ms'] or {}).get('p99'))} | "
                     f"{number(interval.get('p99'))}/{number(interval.get('max'))} | "
                     f"{number((s['receive_interval_ms'] or {}).get('p99'))} | {s['statuses']} |")
    table += ['', '100-cycle pilot의 p99는 사실상 상위 두 표본에 좌우된다. 독립 표본을 가정해도 '
              '100개 표본의 max가 모집단 p99 아래일 확률은 0.99^100≈36.6%다. '
              'timer cycle에는 자기상관도 있으므로 이 표를 안정된 장기 tail 추정으로 해석하지 않는다. '
              '3000-cycle 실행도 1회이며 조건별 3회 반복 및 실제 전체 stack 부하는 미완료다.', '',
              'A=batch/common MC, B=split/1 worker, C=split/2, D=split/5. '
              'A/B/C/D는 timeout=10s로 전체 계산을 관측했고 입력이 만료된 결과는 폐기했다. '
              'production_*는 YAML의 0.18s timeout을 적용한다. K=100, N=10, dt=.2, '
              '정적 검사/대표 재검사와 ε=.05를 유지했다. '
              'A/B/C/D 뒤 odom source stamp 및 moving prediction horizon 검사를 보강했다. '
              '각 run의 source_sha256.json과 실제 parameter dump가 정확한 실행 버전이다.', '',
              'ROS 입력은 고정 상태·경로·지도와 반복되는 합성 예측이며 header age의 DDS jitter는 남는다. '
              '동일 snapshot의 B/C/D 수치 결정성은 별도 단위 테스트에서 검증했다. '
              '실제 MPC·센서·인지 동시 부하, 실차 추종/충돌률은 측정하지 않았다.', '']
    abcd = [(s, rows) for s, rows in runs if s['run_id'].startswith('ros_abcd/')]
    if abcd:
        fig, axes = plt.subplots(1, 2, figsize=(11, 4))
        colors = dict(A='#4c78a8', B='#f58518', C='#54a24b', D='#e45756')
        for ax, field, title in ((axes[0], 'all_callback_ms', 'Full callback p99'),
                                  (axes[1], 'scheduled_response_ms', 'Scheduled response p99')):
            for index, (s, _) in enumerate(abcd):
                condition = s['run_id'].split('/')[1][0]
                ax.bar(index, s[field]['p99'], color=colors[condition])
            ax.set_xticks(range(len(abcd)), [s['run_id'].split('/')[1] for s, _ in abcd], rotation=60, ha='right')
            ax.axhline(200, color='black', linestyle='--', linewidth=1)
            ax.set(ylabel='ms', title=title)
        save_figure(fig, output, 'ros_p99_comparison')
        fig, ax = plt.subplots(figsize=(9, 4))
        labels, stages = [], []
        for s, rows in abcd:
            computed = [r for r in rows if r.get('optimizer_ms') is not None]
            if not computed:
                continue
            stage = []
            for r in computed:
                anchor = (r.get('stage_ms') or {}).get('anchor', 0.)
                primary = (r.get('stage_ms') or {}).get('primary_wall', 0.)
                final = r['representative_ms'] or 0.
                other = max(0., r['optimizer_ms']-anchor-primary-final)
                overhead = max(0., r['callback_ms']-r['optimizer_ms'])
                stage.append([anchor, primary, final, other, overhead])
            labels.append(s['run_id'].split('/')[1])
            stages.append(np.mean(stage, axis=0))
        stages = np.asarray(stages)
        bottom = np.zeros(len(labels))
        for i, name in enumerate(('Anchor/preprocess', 'Primary guide wall', 'Representative recheck', 'Optimizer other', 'ROS/adapter')):
            ax.bar(range(len(labels)), stages[:, i], bottom=bottom, label=name)
            bottom += stages[:, i]
        ax.set_xticks(range(len(labels)), labels, rotation=60, ha='right')
        ax.set(ylabel='mean ms (computed cycles)', title='Non-overlapping wall stages')
        ax.legend(fontsize=8, loc='upper left', bbox_to_anchor=(1., 1.))
        save_figure(fig, output, 'planner_stages')
        table += ['![ROS 비교](figures/ros_p99_comparison.png)', '',
                  '![단계별 wall time](figures/planner_stages.png)', '',
                  '단계 그래프는 계산 cycle의 mean wall time이다. worker별 시간 합을 stack에 더하지 않았다.', '']
    final_mc = root/'mc_final/mc_summary.json'
    if final_mc.exists():
        mc = read_json(final_mc)
        table += ['## MC 수치 정확도', '',
                  '최종 밀도 4096, 최소 4096, 최대 131072. 새 평가 seed 200..215. '
                  '등방 Gaussian은 비중심 χ² CDF, 이방 혼합은 독립 polar Gauss–Legendre 적분 '
                  '(80/160 order 비교)을 기준으로 했다. 사전 기준은 [protocol.md](protocol.md).', '',
                  '| span m | MAE | 절대오차 p95 | max | false-safe / unsafe | margin false-safe / unsafe | empty |',
                  '|---:|---:|---:|---:|---:|---:|---:|']
        for s in mc['summaries']:
            table.append(f"| {s['span']} | {s['mae']:.6f} | {s['absolute_p95']:.6f} | {s['absolute_max']:.6f} | "
                         f"{s['false_safe_count']}/{s['unsafe_denominator']} | "
                         f"{s['false_safe_margin_count']}/{s['unsafe_margin_denominator']} | {s['empty_circles']} |")
        table += ['', 'false-safe는 reference>.05인데 estimate≤.05, margin 분모는 reference≥.06이다. '
                  '분모는 수치 평가 사례 수이며 주행 사고 횟수가 아니다. '
                  '2048은 이전 평가에서 max=.01838로 사전 .015 기준을 실패해 제외했다. '
                  '4096의 최종 fixed-distribution suite는 기준을 통과했다.', '']
    domains = root/'mc_domains/domains_summary.json'
    if domains.exists():
        table += ['| 실제 후보 yaw | MC 영역 | MAE | 절대오차 p95 | max | false-safe / unsafe | margin false-safe / unsafe |',
                  '|---:|---|---:|---:|---:|---:|---:|']
        for s in read_json(domains)['summaries']:
            table.append(f"| {s['yaw']} | {s['backend']} | {s['mae']:.6f} | {s['absolute_p95']:.6f} | {s['absolute_max']:.6f} | "
                         f"{s['false_safe']}/{s['unsafe_denominator']} | {s['false_safe_margin']}/{s['unsafe_margin_denominator']} |")
        table += ['', '동일한 실제 rollout 후보, 서로 다른 장애물 반경·공분산 폭·2개 혼합모드를 사용했다. '
                  '**ε 바로 근처 false-safe가 batch와 split 모두 4/2096 발생했다(yaw=0 조건).** '
                  'margin .01 그룹에서는 관측하지 않았지만 안전 인증이 아니다. '
                  '추정 확률의 seed 변동과 유한 표본 오차가 남으며 calibration/폐루프 사고율은 별도다.', '']
        with (root/'mc_domains/domains_raw.csv').open() as stream:
            domain_rows = list(csv.DictReader(stream))
        fig, ax = plt.subplots(figsize=(7, 4))
        for backend in ('batch', 'split'):
            errors = [abs(float(r['error'])) for r in domain_rows if r['backend'] == backend]
            ax.hist(errors, bins=np.geomspace(1e-9, .02, 45), histtype='step', label=backend)
        ax.set(xscale='log', ylabel='candidate/time/seed cases', xlabel='absolute probability error', title='MC domain accuracy on identical candidates')
        ax.legend()
        save_figure(fig, output, 'mc_domain_error')
        table += ['![MC 오차](figures/mc_domain_error.png)', '']
    navigation = root/'navigation/closed_loop_summary.json'
    if navigation.exists():
        nav = read_json(navigation)
        table += ['## 합성 폐루프', '',
                  '20개 기하/자세/예측 조건 × seed 101..103 × standard/guide. '
                  '실제 MPC를 포함하지 않은 ideal kinematic execution이며, 도달은 위치 오차<.15m로 판정한다. '
                  'goal_near 결과가 최종 자세·속도 settle을 검증하는 것은 아니다. blocked는 도달 불가능한 대조 조건이다.', '',
                  '| 상황 | 방법 | 도달/시행 | 충돌 | 평균 도달/실패 시간 s | 평균 경로 오차 m | heading 오차 rad | guide 전환 |',
                  '|---|---|---:|---:|---:|---:|---:|---:|']
        results = nav['results']
        for name in dict.fromkeys(r['environment'] for r in results):
            for method in ('standard', 'guide'):
                group = [r for r in results if r['environment'] == name and r['method'] == method]
                table.append(f"| {name} | {method} | {sum(r['success'] for r in group)}/{len(group)} | "
                             f"{sum(r['collision'] for r in group)} | {np.mean([r['failure_inclusive_s'] for r in group]):.2f} | "
                             f"{np.mean([r['mean_path_error_m'] for r in group]):.3f} | "
                             f"{np.mean([r['mean_heading_error_rad'] for r in group]):.3f} | "
                             f"{sum(r['guide_switches'] for r in group)} |")
        failures = [r for r in results if not r['success']]
        (root/'navigation/failure_cases.json').write_text(json.dumps(failures, indent=2))
        table += ['', '최소 clearance, 입력 변화, 정지 시간, 위험 초과, 예외 사유는 '
                  '[closed_loop_summary.json](navigation/closed_loop_summary.json), '
                  '실패 상세는 [failure_cases.json](navigation/failure_cases.json)에 있다. '
                  '튜닝 seed 3 및 상황 집합과 최종 seed 101..103을 분리했지만, 수동 scene의 '
                  '전반적인 일반화·실차 성능 우월성을 주장할 수 없다.', '']
        raw = [json.loads(line) for line in (root/'navigation/closed_loop_raw.jsonl').read_text().splitlines()]
        fig, axes = plt.subplots(2, 3, figsize=(11, 6))
        for ax, name in zip(axes.flat, ('offpath', 'yaw180', 'detour', 'corner', 'crossing5', 'headon')):
            for method in ('standard', 'guide'):
                rows = [r for r in raw if r['environment'] == name and r['method'] == method and r['seed'] == 101]
                if rows:
                    xy = np.asarray([r['state'][:2] for r in rows])
                    ax.plot(xy[:, 0], xy[:, 1], label=method)
            ax.set(title=name, xlabel='x m', ylabel='y m', aspect='equal')
            ax.legend(fontsize=7)
        save_figure(fig, output, 'navigation_paths')
        table += ['![합성 폐루프 경로](figures/navigation_paths.png)', '']
    (root/'results.md').write_text('\n'.join(table)+'\n')
    print(f'wrote {root / "results.md"}; {len(runs)} ROS runs')


if __name__ == '__main__':
    main()
