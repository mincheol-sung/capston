"""Render the selected gap proposal design; no synthetic measurement graphics."""
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import font_manager
from matplotlib.patches import FancyArrowPatch, FancyBboxPatch


ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "docs/gap_multiscene_2026-11-04/00_selected_design_flow.png"
FONT = "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"
font_manager.fontManager.addfont(FONT)
plt.rcParams.update({
    "font.family": font_manager.FontProperties(fname=FONT).get_name(),
    "axes.unicode_minus": False,
})

STEPS = [
    ("입력 정렬", "현재 로봇 좌표계로 입력을 맞춘다", [
        "앞·뒤 LiDAR + odometry / TF + 지도 → 장애물·빈 공간·미확인 영역 구분",
        "기존 기준 제어입력열(nominal) + global path + 사람 이동 예측 준비",
        "미확인 영역은 통과 불가로 처리하며, 지도는 관측 빈틈을 보완한다.",
    ], "#466380"),
    ("공간 표현", "방향별로 몸체가 얼마나 진입할 수 있는가?", [
        "1° 간격으로 d_free(θ) = 충돌 없이 직진할 수 있는 로봇 중심의 거리 계산",
        "로봇 반경 0.20 m + 여유 0.05 m 및 격자·궤적 검사 보수성 반영",
        "LiDAR 원시 거리 그대로가 아니라, 로봇 몸체가 통과 가능한 깊이를 사용한다.",
    ], "#187c86"),
    ("gap 분할", "연결된 열린 방향을 묶고, 깊이 골짜기에서 나눈다", [
        "d_free ≥ 0.5 m인 연속 각도 구간 생성 → 평활화한 깊이 profile의 골짜기로 분리",
        "peak prominence ≥ 0.25 m / peak 간격 ≥ 20° / 구간 폭 ≥ 6°",
        "같은 방향을 0.5·1.0·1.5 m별 gap으로 복제하지 않는다. 충돌 검사는 원래 지도로 한다.",
    ], "#187c86"),
    ("seed 생성", "gap마다 최대 3개 방향을 가볍게 비교한다", [
        "① 각도 중앙   ② 가장 깊은 방향   ③ 경로 방향을 gap 안으로 제한한 방향",
        "목표 거리를 빈 공간과 2초 도달 범위로 조절하고, 속도·가속도·입력 제약 적용",
        "각 방향의 10-step 제어입력열을 생성한다. 이 단계에서 MPPI를 3개씩 돌리지는 않는다.",
    ], "#187c86"),
    ("가치 판단 ①", "실행 불가하거나 nominal과 겹치는 seed를 제외한다", [
        "제외: 몸체 궤적 충돌 또는 미확인 영역 진입 / 최종 목표 오차 > 0.35 m",
        "nominal 중복: 위치 RMS 차이 < 0.22 m 이면서 마지막 3점 평균 차이 < 0.30 m",
        "nominal이 정적 충돌하면 nominal 중복 조건은 해제한다.",
    ], "#ad681c"),
    ("가치 판단 ②", "기존 MPPI 비용으로 대표를 고르고, 최대 3개 gap 선택", [
        "J_seed = 기존 공통 비용 + 정적 장애물 비용 → gap마다 가장 낮은 비용의 seed 1개 유지",
        "전체 대표를 비용 순으로 선택 / 이미 선택한 seed와 RMS < 0.25 m면 제외 / 최대 3개",
        "별도 경로 근접성 R 점수 없음. nominal보다 seed 비용이 높다는 이유만으로 기각하지 않음.",
    ], "#ad681c"),
    ("MPPI 탐색", "nominal과 선택한 gap 주변에서 총 500개 rollout", [
        "선택 gap 수 g = 0~3 → gap마다 100개 / nominal은 500 − 100g개",
        "g=3: nominal 200 + gap별 100   |   g=0: nominal 500",
        "각 대표 제어입력열 주변을 sampling하고 기존 MPPI 비용·MC 평가로 최적화한다.",
    ], "#5458a6"),
    ("최종 선택", "공통 안전 재평가 후 실행할 제어입력을 선택한다", [
        "nominal·gap의 최적화 결과를 기존 공통 MC 조건에서 재평가",
        "안전한 이동 후보 중 기존 비용 기준으로 선택 / 안전한 이동 후보가 없으면 Stop 선택",
        "첫 제어입력 적용 → 다음 주기에 갱신된 입력으로 반복",
    ], "#5458a6"),
]


def main():
    fig = plt.figure(figsize=(16, 19), dpi=170, facecolor="#f3f6fa")
    ax = fig.add_axes([0, 0, 1, 1])
    ax.set(xlim=(0, 1), ylim=(0, 1))
    ax.axis("off")
    ax.text(.055, .968, "LiDAR gap 기반 DRA-MPPI", fontsize=29,
            weight="bold", color="#172c43", va="top")
    ax.text(.055, .932, "선택한 설계의 전체 FLOW", fontsize=21,
            weight="bold", color="#187c86", va="top")
    ax.text(.055, .904,
            "공간은 깊이 골짜기로 분할  ·  탐색 가치는 실행 가능성 + 궤적 차이 + 기존 비용으로 판단",
            fontsize=13.5, color="#4c6176", va="top")
    top, height, stride = .870, .092, .104
    for i, (label, title, lines, color) in enumerate(STEPS):
        y = top - i * stride
        ax.add_patch(FancyBboxPatch((.05, y-height), .90, height,
                     boxstyle="round,pad=0.005,rounding_size=0.009",
                     facecolor="white", edgecolor="#d9e2ec", linewidth=1))
        ax.add_patch(FancyBboxPatch((.061, y-height+.010), .133, height-.020,
                     boxstyle="round,pad=0.004,rounding_size=0.006",
                     facecolor=color, edgecolor="none"))
        ax.text(.1275, y-.023, f"{i+1:02d}", fontsize=23, weight="bold",
                color="white", ha="center", va="center")
        ax.text(.1275, y-.058, label, fontsize=13.5, weight="bold",
                color="white", ha="center", va="center")
        ax.text(.216, y-.010, title, fontsize=16, weight="bold",
                color="#172c43", va="top")
        for k, line in enumerate(lines):
            ax.text(.216, y-.034-k*.018, line, fontsize=11.6,
                    color="#3d5268", va="top")
        if i < len(STEPS)-1:
            ax.add_patch(FancyArrowPatch((.50, y-height-.005), (.50, y-stride+.005),
                         arrowstyle="-|>", mutation_scale=13,
                         linewidth=1.5, color="#6c8197"))
    ax.text(.055, .032,
            "개발 설계안 · 수치는 실험 기반 초기값 · 실제 주행 성능과 안정적인 5 Hz는 아직 검증되지 않음",
            fontsize=12, color="#8c4b20", va="center")
    ax.text(.055, .014,
            "근거: 실제 bag 3개, 유효 장면 127개 / 방향 비용 비교 방식의 offline 254회 중 6회가 200 ms 초과",
            fontsize=10.5, color="#65768a", va="center")
    OUT.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(OUT, dpi=170, facecolor=fig.get_facecolor())
    plt.close(fig)
    print(OUT)


if __name__ == "__main__":
    main()
