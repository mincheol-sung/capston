#!/usr/bin/python3
"""2026-11-24 마찰 상한 옮김 시뮬 결과 표. runs/*.json (pf_rec.py) → 판별 두 번의 값."""
import glob, json, os, re
HERE = os.path.dirname(os.path.abspath(__file__))
rows = {}
for p in sorted(glob.glob(os.path.join(HERE, "runs", "*.json"))):
    lab = os.path.basename(p)[:-5]
    d = json.load(open(p))
    m = re.match(r"(.+)_r(\d)(\.fault\d)?$", lab)
    rows.setdefault(m.group(1), []).append((m.group(2) + (m.group(3) or ""), d))
keys = [("moving_rms_mm", "위치 오차 RMS mm"), ("moving_max_mm", "최대 mm"), ("cont_rms_mm", "경로 벗어남 RMS mm"), ("yaw_rms_deg", "헤딩 RMS °"),
        ("slip_frac50", "큰 슬립 %"), ("slip_frac10", "슬립>0.1 %"), ("slip_rms", "슬립 RMS"), ("tau_max", "토크 최대"), ("drive_time", "주행 s")]
print("| 판 | 회 | " + " | ".join(k[1] for k in keys) + " | 멈춤 |")
print("|---|---|" + "---|" * (len(keys) + 1))
for name in ["Cfc", "Cnofc_nod", "Cnofc", "Cshift_nod", "Cshift", "Cshift_ground", "PPnofc"]:
    for rep, d in rows.get(name, []):
        vals = []
        for k, _ in keys:
            v = d.get(k)
            vals.append("—" if v is None else (f"{100*v:.1f}" if k.startswith("slip_frac") else f"{v:.3f}" if k in ("slip_rms", "tau_max") else f"{v:.1f}" if "mm" in k or k == "drive_time" else f"{v:.2f}"))
        print(f"| {name} | {rep} | " + " | ".join(vals) + f" | {d.get('fault') or ''} |")
