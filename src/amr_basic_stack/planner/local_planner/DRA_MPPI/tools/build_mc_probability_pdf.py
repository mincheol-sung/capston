#!/usr/bin/env python3
"""Build Korean 16:9 presentation PDF from saved matched MC accuracy results."""
import csv
import hashlib
import json
from pathlib import Path
import shutil

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages
from matplotlib.font_manager import FontProperties
from matplotlib.patches import FancyBboxPatch
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT/'docs/mc_budget_theta_2026-10-23'
OUT = ROOT/'docs/mc_probability_presentation_2026-10-26'
REFERENCE_TIMING = ROOT/'docs/mc_reference_timing_2026-10-27/timing.json'
PREDICTOR_TIMING = ROOT/'docs/predictor_integration_comparison_2026-10-27/results.json'
FONT = str(OUT/'fonts/NanumGothic-Regular.ttf')
BOLD = str(OUT/'fonts/NanumGothic-Bold.ttf')
plt.rcParams['pdf.fonttype'] = 42
FP = FontProperties(fname=FONT)
FB = FontProperties(fname=BOLD)
INK = '#172b45'
BLUE = '#2468bd'
MUTED = '#52657a'
TEAL = '#087f8c'
RED = '#be3b46'
PAGES = []


def display(value):
    for source, target in {'①':'1.', '②':'2.', '③':'3.', '−':'-', 'σ':'표준편차 ', 'ε':'epsilon', 'Σ':'합'}.items():
        value = value.replace(source, target)
    return value


def text(fig, x, y, value, size=16, color=INK, bold=False, va='top', **kwargs):
    return fig.text(x, y, display(value), fontsize=size, color=color,
                    fontproperties=FB if bold else FP, va=va, **kwargs)


def box(fig, x, y, w, h, color='#edf3fa', edge=None):
    fig.add_artist(FancyBboxPatch((x, y), w, h, transform=fig.transFigure,
        boxstyle='round,pad=0.008,rounding_size=0.014',
        facecolor=color, edgecolor=edge or color, zorder=-1))


def page(section, title, subtitle=''):
    fig = plt.figure(figsize=(16, 9), facecolor='white')
    fig.subplots_adjust(0, 0, 1, 1)
    text(fig, .045, .963, section, 12, BLUE, True)
    text(fig, .045, .91, title, 27, INK, True)
    if subtitle:
        text(fig, .045, .842, subtitle, 13, MUTED)
    text(fig, .045, .028, 'MC 확률 추정 검증  |  측정 2026.10.23 / 10.27 · 업데이트 2026.10.28', 9, MUTED)
    text(fig, .955, .028, f'{len(PAGES)+1:02d}', 10, MUTED, ha='right')
    PAGES.append(fig)
    return fig


def table(fig, rows, cols, rect, widths=None, size=14):
    ax=fig.add_axes(rect);ax.axis('off')
    tab=ax.table(cellText=[[display(v) for v in row] for row in rows], colLabels=[display(v) for v in cols], cellLoc='center',
                 colWidths=widths, bbox=[0, 0, 1, 1])
    tab.auto_set_font_size(False)
    for (r, c), cell in tab.get_celld().items():
        cell.get_text().set_fontproperties(FB if r==0 else FP)
        cell.get_text().set_fontsize(size)
        cell.set_edgecolor('#ffffff')
        cell.set_linewidth(2)
        cell.set_facecolor(INK if r==0 else ('#edf3fa' if r%2 else '#f6f8fb'))
        cell.get_text().set_color('white' if r==0 else INK)
    return tab


def render_comparison(multiplier, accuracy):
    rows=list(csv.DictReader((SOURCE/f'accuracy_candidates_x{multiplier}.csv').open()))
    fig, axes=plt.subplots(1, 2, figsize=(12, 5.2), constrained_layout=True)
    for ax, profile, title in zip(axes, ['guide', 'final'], ['Guide: K500 candidates', 'Final: representatives + Stop']):
        group=[r for r in rows if r['profile']==profile]
        points=np.array([(float(r['reference'])*100, float(r['estimate'])*100) for r in group])
        points=points[points[:, 0]<=20]
        if len(points)>20000:
            rng=np.random.default_rng(817)
            points=points[rng.choice(len(points), 20000, replace=False)]
        ax.scatter(points[:, 0], points[:, 1], s=4, alpha=.2, color=BLUE, rasterized=True)
        bad=np.array([(float(r['reference'])*100, float(r['estimate'])*100) for r in group
                      if .05<float(r['reference'])<=.2 and float(r['estimate'])<=.05])
        if len(bad):
            ax.scatter(bad[:, 0], bad[:, 1], s=14, alpha=.7, color=RED,
                       label='Reference >5%, MC <=5%', rasterized=True)
        ax.plot([0,20], [0,20], color=INK, lw=1.2, label='Perfect estimate')
        ax.axvline(5, color=RED, ls='--');ax.axhline(5, color=RED, ls='--')
        stat=accuracy['actual'][profile]
        ax.text(.98, .04, f"All-step MAE {stat['mae']*100:.2f} pp\nAbsolute error p95 {stat['absolute_p95']*100:.2f} pp",
            transform=ax.transAxes, ha='right', fontsize=10,
            bbox=dict(facecolor='white', alpha=.92, edgecolor='#dddddd'))
        ax.set(xlim=(0,20), ylim=(0,20), xlabel='Reference collision probability (%)',
               ylabel='MC estimate (%)', title=title)
        ax.grid(alpha=.15);ax.legend(loc='upper left', fontsize=8)
    fig.suptitle(f"{multiplier}x MC budget: density {124*multiplier} samples/m²; threshold 5%", fontsize=14)
    folder=OUT/('original_1x' if multiplier==1 else 'modified_8x');folder.mkdir(parents=True, exist_ok=True)
    fig.savefig(folder/'probability_comparison.png', dpi=180)
    fig.savefig(folder/'probability_comparison.svg')
    plt.close(fig)
    return folder/'probability_comparison.png'


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    accuracy=json.loads((SOURCE/'accuracy.json').read_text())
    timing=json.loads((SOURCE/'timing.json').read_text())
    a={r['multiplier']:r for r in accuracy}
    original=render_comparison(1, a[1]);modified=render_comparison(8, a[8])
    # Keep the originally requested historical PNGs, too. PDF uses matched repeats.
    old=ROOT/'docs/current_mc_accuracy_2026-10-22'
    history=OUT/'historical_png';history.mkdir(exist_ok=True)
    shutil.copyfile(old/'probability_comparison_density124.png', history/'probability_comparison_original_8repeats.png')
    shutil.copyfile(old/'probability_comparison.png', history/'probability_comparison_modified.png')

    fig=page('01  실험 목적', '충돌확률을 MC로 얼마나 정확히 계산하는가?',
             '기존 표본 예산과 8배 예산을 동일 Gaussian 기준 확률에 비교')
    box(fig,.045,.48,.91,.29)
    text(fig,.07,.738,'검증 질문',16,BLUE,True)
    text(fig,.07,.68,'① 적은 표본 때문에 5% 위험 기준을 잘못 판단하는가?\n② 표본을 8배 늘리면 오차가 얼마나 줄어드는가?\n③ 정확도 개선 뒤에도 5Hz 계산 예산을 만족하는가?',20,linespacing=1.7)
    box(fig,.045,.17,.44,.25,'#f2f6fb');box(fig,.515,.17,.44,.25,'#eaf5f3')
    text(fig,.065,.39,'비교 기준',17,BLUE,True)
    text(fig,.065,.328,'Gaussian 분포의 원형 충돌영역 적분\n등방: CDF / 이방: 고정밀 수치 적분',16,linespacing=1.6)
    text(fig,.535,.39,'MC 추정값',17,TEAL,True)
    text(fig,.535,.328,'동일 분포·동일 후보 위치를\nscrambled Halton 공간 표본으로 적분',16,linespacing=1.6)
    text(fig,.065,.222,'기준: 원형 충돌영역 안의 확률밀도 적분',12,MUTED)
    text(fig,.535,.222,'MC: 면적(B)/M × Σ[원 내부 지시값 × 확률밀도]',12,MUTED)
    text(fig,.05,.11,'범위: Gaussian 모델 내부의 수치 정확도. 실세계 충돌 빈도나 사람 예측모델의 보정 정확도는 측정하지 않음.',12,RED)

    fig=page('02  실험 방법', '같은 후보·같은 분포·같은 반복 횟수로 비교',
             '본 PDF의 전후 그림과 수치는 동일한 4회 scramble 반복을 사용')
    rows=[['후보 / horizon','5 guide × 100 = 500 후보, 최종 유효 대표 최대5개 + Stop / 10step × 0.2s'],
          ['로봇 / 경로','위치 (0, 0.2m), yaw 0°·−70°·180° / 전역경로 (0,0) → (3,0)m'],
          ['동적 장애물','횡단1 / 정면접근5 / 평행이동10 / 다중모드20, 각 장애물 2-mode'],
          ['확률 분포','그림: 등방 σ 0.08·0.2·0.6m, mode 가중치 0.6/0.4'],
          ['충돌 조건','로봇 반경0.20 + 장애물 반경0.25 = 0.45m / 위험 기준 ε=0.05'],
          ['비교 개수 / 단계','guide 240,000 + final 2,880 candidate-step / 1·2·4·8·16·32배']]
    table(fig, rows, ['항목','실험 조건'], [.045,.33,.91,.45], [.19,.81], 13)
    text(fig,.05,.279,'별도 고정점 시험: 40개 분포 × 영역 크기5종 × scramble8회. 회전 이방·분리된 혼합분포도 포함.',13)
    text(fig,.05,.225,'독립 기준: 등방 ncx2 CDF / 이방 polar 적분 80→160 refinement. 여러 장애물은 독립 가정으로 결합.',13)
    text(fig,.05,.17,'중요: 위험비용·정적검사를 끄고 후보 형상만 고정한 뒤 확률을 평가. 최종 운용 선택 경로의 분포가 아님.',13,RED)
    text(fig,.05,.11,'장비: Intel Core 5 210H, RAM 약7.3GiB / ROS Jazzy / Python3.12.3, NumPy1.26.4, SciPy1.11.4',11,MUTED)
    text(fig,.05,.076,'그림의 각 점은 한 시각의 확률. 같은 후보의 시각·scramble 반복이 포함되므로 독립 주행 횟수가 아님.',11,MUTED)

    fig=page('02  실험 방법 · MC 예산', '대표 궤적의 MC 확률은 새로 계산한다',
             '현재 YAML: monte_carlo / guide_evaluation=split / adaptive_halton_mc=true')
    box(fig,.045,.49,.43,.27);box(fig,.525,.49,.43,.27,'#eaf5f3')
    text(fig,.065,.735,'1차: guide별 후보 평가',18,BLUE,True)
    text(fig,.065,.676,'각 guide의 100후보를 감싸는 사각형 B(g,t)\n각 timestep에서 guide 전용 표본을 추출\n같은 guide의 100후보가 그 표본을 공유',16,linespacing=1.6)
    text(fig,.545,.735,'최종: 대표 최대5개 + Stop',18,TEAL,True)
    text(fig,.545,.676,'가중평균 제어 → 제약 적용 → 다시 rollout\n유효 대표와 Stop 전체의 사각형 B(final,t)\n별도 estimator의 새 표본을 함께 공유',16,linespacing=1.6)
    text(fig,.495,.635,'→',30,BLUE,ha='center')
    text(fig,.05,.443,'MC 개수 = ceil(밀도 × 사각형 면적), 최소·최대 예산으로 제한   [timestep마다 계산]',17,bold=True)
    table(fig,[['기존 1배','124','128 ~ 1,024','256 ~ 4,096'],
               ['수정 8배','992','1,024 ~ 8,192','2,048 ~ 32,768']],
          ['예산','밀도 [표본/m²]','guide 표본/step','final 표본/step'],[.045,.245,.91,.155],size=14)
    text(fig,.05,.205,'대표의 확률·비용은 후보 확률의 평균이 아님. 1차 표본도 그대로 재사용하지 않음.',15,RED,True)
    text(fig,.05,.153,'8배 실제 후보 시험: guide 1,024~3,396개/step, final 2,048~4,958개/step. 후보당 따로 뿌리는 개수가 아님.',12)
    text(fig,.05,.109,'guide별 seed=17+1009+g, final seed=17+2003. estimator별 Halton 시퀀스는 호출에 따라 진행.',11,MUTED)
    text(fig,.05,.072,'adaptive 모드에서 fixed mc_samples=2048은 사용하지 않음. final 표본 개수에 대표 수를 곱하는 방식도 아님.',11,MUTED)

    for mult, image, label in [(1,original,'기존 MC 예산: 밀도124'),(8,modified,'수정 MC 예산: 밀도992 (8배)')]:
        fig=page('03  결과 및 해석 · 확률 비교',label,
                 '가로축: 기준 적분 확률 / 세로축: MC 추정 확률 / 검은 대각선: 정확히 일치')
        ax=fig.add_axes([.045,.23,.91,.57]);ax.imshow(plt.imread(image));ax.axis('off')
        g=a[mult]['actual']['guide'];f=a[mult]['actual']['final']
        text(fig,.05,.206,f"오차 p95: guide {g['absolute_p95']*100:.2f}%p / final {f['absolute_p95']*100:.2f}%p   |   final 평균 절대 오차 {f['mae']*100:.2f}%p",17,bold=True)
        text(fig,.05,.146,'빨간 점선 = 위험 기준5%. 오른쪽 아래 빨간 점 = 기준은 5% 초과인데 MC는 5% 이하로 판단한 step.',13,RED)
        text(fig,.05,.101,'표시 범위0~20%; guide 파란 점은 최대2만 개로 표시. 통계는 전체값 사용, 표시 범위의 false-safe는 모두 표시.',11,MUTED)
        text(fig,.05,.066,'위험한 후보도 시험에 포함됨. 20% 점이 많다는 사실은 실제 선택 경로에서 20% 위험이 자주 발생한다는 뜻이 아님.',11,MUTED)

    fig=page('03  결과 및 해석 · 정확도', '8배 예산에서 final 오차 p95가 5.77 → 1.07%p',
             '절대 오차: |MC 확률 − 기준 확률|. 예: 5% → 6%는 1%p 오차')
    rows=[]
    for mult in [1,8]:
        for profile,label in [('guide','guide 1차'),('final','final 대표/Stop')]:
            s=a[mult]['actual'][profile]
            rows.append([f'{mult}배 · {label}',f"{s['mae']*100:.2f}%p",f"{s['absolute_p95']*100:.2f}%p",f"{s['absolute_max']*100:.2f}%p",
                         f"{s['false_safe']:,}/{s['unsafe_denominator']:,}"])
    table(fig,rows,['예산 · 단계','평균 오차','오차 p95','최대 오차','false-safe / 위험 step'],[.045,.405,.91,.36],[.23,.14,.14,.14,.35],13)
    box(fig,.045,.17,.44,.18);box(fig,.515,.17,.44,.18,'#eaf5f3')
    text(fig,.065,.325,'5% 근처 오분류는 남음',17,RED,True)
    text(fig,.065,.268,'final false-safe: 12 → 2건 / 1,488 위험 step\n기준≥6%만 보면: 6 → 0건 / 1,452 step',14,linespacing=1.7)
    text(fig,.535,.325,'수치 정확도 개선 확인',17,TEAL,True)
    text(fig,.535,.268,'final 오차 p95 약81.5% 감소\n하지만 최대 오차 3.98%p로 남음',15,linespacing=1.7)
    text(fig,.05,.108,'false-safe = 기준 확률>5%이고 MC≤5%. step 오분류이며 최종 경로 통과·실제 충돌 횟수가 아니다.',12,MUTED)
    text(fig,.05,.067,'별도 고정점 시험(등방·회전 이방·혼합)의 final p95: 3.64 → 0.67%p (underresolved reject 평가 제외).',11,MUTED)

    fig=page('03  결과 및 해석 · 계산 비용', '정확도는 개선되지만 계산량도 함께 증가',
             'K500 / N10 / worker2 / 장애물20 ×2mode / 회전 이방 분포 / 정적 검사·최종 재평가 포함')
    rows=[]
    for mult in [1,2,4,8,16,32]:
        value=next(r for r in timing if r['multiplier']==mult)['timing'][-1]
        f=a[mult]['actual']['final']
        rows.append([f'{mult}배',f"{f['absolute_p95']*100:.2f}%p",f"{value['mean_ms']:.1f}ms",f"{value['max_ms']:.1f}ms",
                     f"{value['primary_samples']:,} / {value['final_samples']:,}"])
    table(fig,rows,['표본 예산','final 오차 p95','모듈 평균','모듈 최대','1차 / final 총 표본·주기'],[.045,.345,.91,.43],[.12,.17,.17,.17,.37],13)
    box(fig,.045,.13,.91,.16,'#eaf5f3')
    text(fig,.065,.27,'8배 ROS pilot: 콜백 p99 137.16ms / 최대139.55ms / timeout 0회',19,TEAL,True)
    text(fig,.065,.21,'warmup10 + 측정50주기, 운용 timeout180ms, 이동 reference 정상50회. 16·32배는 장애물20에서 200ms 초과.',12)
    text(fig,.05,.097,'모듈 시간은 2warmup+5측정, timeout10s로 완주 시간을 관찰. ROS p99와 다른 계층의 측정이다.',11,MUTED)
    text(fig,.05,.061,'ROS fixture: 위치고정, 10×10m·해상도5cm·경계장애물 지도, 합성 예측. 인지/MPC/모터 부하 없이 command 출력 OFF.',11,MUTED)

    reference=json.loads(REFERENCE_TIMING.read_text())
    predictor=json.loads(PREDICTOR_TIMING.read_text())
    fig=page('03  추가 측정 · 이방 기준 적분과 MC', '고정밀 기준 적분과 비교: MC는 PDF 계산을 공유한다',
             '2026.10.27 측정 | 동일 후보·동일 이방 분포 | 충돌확률 계산만 측정, 전체 controller 시간과 구분')
    rows=[]
    for profile,label in [('guide','후보 500개'),('final','최종 6개')]:
        mc=next(r for r in reference['rows'] if r['family']=='anisotropic' and r['profile']==profile
                and r['method']=='mc' and r['multiplier']==8)
        integral=next(r for r in reference['rows'] if r['family']=='anisotropic' and r['profile']==profile
                      and r['method']=='polar_order160')
        rows.append([label,f"{mc['mean_ms']:.2f} ms",f"{integral['mean_ms']/1000:.2f} 초",
                     f"{integral['mean_ms']/mc['mean_ms']:,.0f}배"])
    table(fig,rows,['평가 대상','8배 MC · 10회 평균','기준 적분 · 전체 1회','MC 속도 우위'],
          [.045,.555,.91,.22],[.23,.26,.28,.23],16)
    box(fig,.045,.32,.91,.18)
    text(fig,.065,.477,'후보 500개에서의 계산량',17,BLUE,True)
    text(fig,.065,.422,'MC8: PDF 387만 회 + 후보-표본 거리 비교 968만 회\n기준 적분: PDF 204.8억 회 (후보·시점마다 160 × 640개 적분점)',16,linespacing=1.6)
    text(fig,.05,.275,'조건: 장애물20 × 2mode, N10·dt0.2s, yaw0, 충돌 반경0.45m, 회전 이방 공분산 [[.05,.025],[.025,.025]].',12)
    text(fig,.05,.229,'장비: Intel Core 5 210H / NumPy1.26.4·SciPy1.11.4 / 1 thread·guide 그룹 순차 평가·캐시 재사용 없음.',12)
    text(fig,.05,.182,'기준은 원 disk_reference(order=160) 그대로 실행. 일부 후보 시간을 환산하지 않음. 80차 검증 시간은 제외.',12)
    text(fig,.05,.132,'주의: 이 기준 적분은 정확도 검증용이다. 같은 오차로 맞춘 비교가 아니며, 모든 적분법이 느리다는 뜻이 아니다.',13,RED,True)
    text(fig,.05,.082,'근거: docs/mc_reference_timing_2026-10-27/timing.json · tools/benchmark_mc_reference_time.py',10,MUTED)

    fig=page('03  추가 측정 · 효율적인 이방 직접 적분', '효율적인 적분과 비교하면 평가할 후보 수가 중요하다',
             '조건부 Gaussian 1차원 적응 적분: 한 축은 정규 CDF, 나머지 축만 수치 적분 (gaussian_disk.py)')
    rows=[]
    for scene,label in [('crossing','횡단'),('headon','정면 접근'),('turning','회전')]:
        cells=[label]
        for profile in ('guide','final'):
            group=[r for r in predictor['timings'] if r['scene']==scene and r['obstacles']==20 and r['profile']==profile]
            mc=next(r for r in group if r['method']=='mc' and r['multiplier']==8)
            direct=next(r for r in group if r['method']=='direct')
            cells.extend([f"{mc['mean_ms']:.1f}",f"{direct['mean_ms']:.1f}"])
        rows.append(cells)
    table(fig,rows,['장면','후보500 MC8','후보500 적분','최종6 MC8','최종6 적분'],
          [.045,.48,.91,.29],[.20,.20,.20,.20,.20],15)
    text(fig,.05,.446,'단위: ms, 준비1회 후 5회 평균. 장애물20 × 3mode(CV/CT/STATIC), N10·dt0.2s, 반경0.45m.',12)
    box(fig,.045,.24,.44,.15);box(fig,.515,.24,.44,.15,'#eaf5f3')
    text(fig,.065,.368,'후보 500개: MC8이 2.4~2.8배 빠름',17,BLUE,True)
    text(fig,.065,.308,'여러 후보가 PDF 계산을 공유하는 이점',14)
    text(fig,.535,.368,'최종 6개: 직접 적분이 5.5~6.4배 빠름',16,TEAL,True)
    text(fig,.535,.308,'적은 후보를 정밀하게 재검사하는 데 유리',14)
    text(fig,.05,.198,'입력: 현재 ImmMotionPredictor와 predictor.yaml로 합성 posterior를 전파. 실측 사람 bag 재생은 아님.',12,RED)
    text(fig,.05,.152,'원 기준 적분 비교(앞 페이지)는 2mode·고정 공분산, 이 페이지는 3mode·시간변화 공분산으로 서로 다른 실험.',12)
    text(fig,.05,.106,'단일 thread·guide 그룹 순차, 충돌확률 계산만 측정. 온라인 객체/모드 필터·정적검사·제약·controller 시간 제외.',11,MUTED)
    text(fig,.05,.069,'근거: docs/predictor_integration_comparison_2026-10-27/results.json · tools/benchmark_predictor_integration.py',10,MUTED)

    fig=page('03  추가 측정 · 정확도와 해석', '1차 탐색 MC + 최종 직접 적분은 검토할 만한 구성',
             '속도만이 아니라 오차를 함께 비교해야 한다. 운용 설정은 이 추가 실험에서 변경하지 않음')
    rows=[]
    for scene,label in [('crossing','횡단'),('headon','정면 접근'),('turning','회전')]:
        row=[label]
        for profile in ('guide','final'):
            stat=next(r for r in predictor['accuracy'] if r['scene']==scene and r['obstacles']==20
                      and r['profile']==profile and r['multiplier']==8)
            row.extend([f"{stat['p95']*100:.3f}",f"{stat['max_error']*100:.3f}"])
        rows.append(row)
    table(fig,rows,['MC8 오차 [%p]','후보500 p95','후보500 최대','최종6 p95','최종6 최대'],
          [.045,.49,.91,.28],[.20,.20,.20,.20,.20],15)
    checks=predictor['validation']
    maximum=max(abs(r['error']) for r in checks)*100
    text(fig,.05,.451,f'직접 적분 독립 검증: {len(checks)}개 지점에서 원 polar 적분과 최대 차이 {maximum:.6f}%p, 미수렴 0건.',13,TEAL,True)
    text(fig,.05,.403,'MC 오차는 전체 후보-step과 직접 적분의 차이. 직접 적분 검증은 선택 지점의 최대차이로 통계 범위가 다름.',12)
    box(fig,.045,.20,.91,.15)
    text(fig,.065,.327,'MC 표본을 늘릴수록 속도 이점은 줄어든다',17,BLUE,True)
    text(fig,.065,.269,'후보500: MC16은 약259~266ms, MC32는614~645ms. 직접 적분은300~340ms.',15)
    text(fig,.05,.162,'판단: 이번 입력에서 다수 후보 탐색은 MC, 소수 최종 검증은 직접 적분이 유망. hybrid 전체 controller는 미검증.',12,RED,True)
    text(fig,.05,.115,'적분 수렴차 기준1e-8과 보수적 여유 사용. 동일 오차로 맞춘 시험·인증된 오차 한계·실차 안전 검증은 아님.',11,MUTED)
    text(fig,.05,.071,'재현 입력·반복별 시간·MC1~32배 오차: docs/predictor_integration_comparison_2026-10-27/',10,MUTED)

    fig=page('03  결과 및 해석 · 결론', '8배는 이번 조건에서 속도와 정확도의 절충안',
             '10월23일 예산 실험의 결론. 10월27일 추가 측정은 앞의 이방 적분 비교 3페이지에 별도 수록')
    box(fig,.045,.52,.91,.26)
    text(fig,.065,.748,'발표 핵심',18,BLUE,True)
    text(fig,.065,.69,'• MC 예산 8배로 final 평균 오차 1.21 → 0.22%p, p95 5.77 → 1.07%p\n• 장애물20 ROS 50주기에서 p99 137ms: 이번 조건의 200ms 계산 예산 충족\n• 대표 궤적은 별도 final 표본으로 확률과 모든 비용을 새로 계산',18,linespacing=1.8)
    text(fig,.05,.461,'최종 선택 시 처리',17,BLUE,True)
    text(fig,.05,.406,'위험비용 = 18 × 시간평균 CP + 100,000 × (CP>5%인 step 수)\n최종 대표는 static hard 검사와 max(CP)≤5% 조건을 통과한 뒤 전환 억제를 적용해 선택.',15,linespacing=1.7)
    text(fig,.05,.294,'해석의 한계',17,RED,True)
    text(fig,.05,.239,'Gaussian 분포의 적분 정확도를 확인한 결과이며 예측모델 보정·실차 안전 보장은 아님.\n전체 궤적의 누적 충돌확률이 아니라 시각별 CP. full-stack 부하·장기 ROS 반복·실차 폐루프는 미검증.',13,linespacing=1.7)
    text(fig,.05,.132,'근거: docs/mc_budget_theta_2026-10-23/accuracy.json · timing.json · ros_x8/summary.json',10,MUTED)
    text(fig,.05,.096,'구현: dra_mppi/guide_conditioned.py (guide estimator / final recost), dynamic_risk.py (영역·적분·결합)',10,MUTED)
    text(fig,.05,.06,'본 PDF 그림은 전후 모두 4회 반복으로 재작성. 최초 1배 원본(8회 반복)은 historical_png 폴더에 별도 보존.',10,MUTED)

    pdf=OUT/'MC_probability_comparison_1x_vs_8x.pdf'
    with PdfPages(pdf) as writer:
        meta=writer.infodict();meta['Title']='MC 충돌확률 추정 검증: 기존 예산 vs 8배 예산'
        meta['Author']='AMR planning experiment';meta['Subject']='실험 목적, 방법, 결과 및 해석'
        for i, fig in enumerate(PAGES, 1):
            writer.savefig(fig)
            fig.savefig(OUT/f'slide_{i:02d}.png', dpi=110)
            plt.close(fig)
    inputs=[SOURCE/'accuracy.json',SOURCE/'timing.json',SOURCE/'ros_x8/summary.json',
            SOURCE/'accuracy_candidates_x1.csv',SOURCE/'accuracy_candidates_x8.csv',
            ROOT/'dra_mppi/guide_conditioned.py',ROOT/'dra_mppi/dynamic_risk.py',ROOT/'config/dra_mppi.yaml',
            REFERENCE_TIMING,PREDICTOR_TIMING,ROOT/'dra_mppi/gaussian_disk.py',
            ROOT/'tools/benchmark_mc_reference_time.py',ROOT/'tools/benchmark_predictor_integration.py']
    (OUT/'manifest.json').write_text(json.dumps(dict(created='2026-10-26',updated='2026-10-28',measurement_date='2026-10-23',
        additional_measurement_dates=['2026-10-27'],pages=len(PAGES),
        figures_matched_repeats=4,pdf=str(pdf.relative_to(ROOT)),source_sha256={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in inputs}),indent=2))
    print(pdf)


if __name__=='__main__':
    main()
