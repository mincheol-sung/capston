#!/usr/bin/python3
"""regress.cpp 를 수정 전·후 라이브러리에 각각 링크해 낸 출력 두 개를 견준다 (2026-11-24, 마찰 상한 옮김을 끈 상태 = 예전과 같은가).
  사용법: regress_compare.py out_old.txt out_new.txt
  만드는 법 (RESULTS_KO.md "끈 상태 회귀"):
    g++ -O2 -std=c++17 -I <mpc_cpp/include> -I/usr/include/eigen3 regress.cpp <build>/libamr_torque_mpc.a <build>/libdaqp_vendored.a -lm -o regress
"""
import collections, sys


def load(p):
    d = collections.defaultdict(list)
    for line in open(p):
        w = line.split()
        d[w[0]].append([float(x) for x in w[2:6]] + [int(w[6]), int(w[7])])
    return d


a, b = load(sys.argv[1]), load(sys.argv[2])
assert a.keys() == b.keys()
worst = 0.0
print(f"{'판':30s} {'스텝':>5s} {'토크 최대 차 [N·m]':>18s} {'비트까지 같은 스텝':>14s}")
for k in a:
    dmax = max(abs(x - y) for ra, rb in zip(a[k], b[k]) for x, y in zip(ra[:4], rb[:4]))
    same = sum(ra[:4] == rb[:4] for ra, rb in zip(a[k], b[k]))
    worst = max(worst, dmax)
    print(f"{k:30s} {len(a[k]):5d} {dmax:18.3e} {same:10d}/{len(a[k])}")
print(f"전체 최대 차 {worst:.3e}  →  {'통과 (≤ 1e-12)' if worst <= 1e-12 else '실패'}")
sys.exit(0 if worst <= 1e-12 else 1)
