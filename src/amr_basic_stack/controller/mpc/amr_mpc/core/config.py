#!/usr/bin/env python3
"""설계 파라미터 YAML 적재 — config/plant.yaml (구조 A), config/plant_wheel.yaml (구조 B).

ROS 에 의존하지 않는다. `python3 -m amr_mpc.body.mpc`, pytest 처럼 노드 밖에서 돌 때도 같은 값을
읽어야 하기 때문이다. 찾는 순서:
  1. 환경변수 AMR_MPC_<FILE>  (예: AMR_MPC_PLANT_YAML, AMR_MPC_PLANT_WHEEL_YAML — 시험에서 다른 값을 꽂을 때)
  2. 설치본 share/amr_mpc/config/        (ros2 run / launch)
  3. 소스 트리 controller/mpc/config/    (직접 실행)
절 구분과 값의 근거는 각 YAML 주석에 있다. 여기서는 적재만 한다.
"""
import os
from pathlib import Path

import yaml

_cache = {}


def _env_name(filename):
    return "AMR_MPC_" + filename.upper().replace(".", "_").replace("-", "_")


def _candidates(filename):
    env = os.environ.get(_env_name(filename))
    if env:
        yield Path(env)
    try:
        from ament_index_python.packages import get_package_share_directory
        yield Path(get_package_share_directory("amr_mpc")) / "config" / filename
    except Exception:
        pass
    yield Path(__file__).resolve().parents[2] / "config" / filename


def path(filename="plant.yaml"):
    for p in _candidates(filename):
        if p.is_file():
            return p
    raise FileNotFoundError(f"{filename} 을 못 찾았다. 찾은 곳: " + ", ".join(str(p) for p in _candidates(filename))
                            + f"   ({_env_name(filename)} 로 직접 줄 수 있다)")


def load(filename="plant.yaml", reload=False):
    """YAML 전체를 dict 로. 프로세스 내 파일당 1회 적재."""
    if filename not in _cache or reload:
        p = path(filename)
        with open(p) as f:
            d = yaml.safe_load(f)
        d["_path"] = str(p)
        _cache[filename] = d
    return _cache[filename]


def section(name, filename="plant.yaml"):
    d = load(filename)
    try:
        return d[name]
    except KeyError:
        raise KeyError(f"{filename} 에 '{name}' 절이 없다 ({d.get('_path')})") from None


if __name__ == "__main__":
    for fn in ("plant.yaml", "plant_wheel.yaml"):
        d = load(fn)
        print(f"[{fn}] {d['_path']}")
        for k in ("model", "geometry", "limits", "tuning", "safety"):
            if k in d:
                print(f"  {k}: " + ", ".join(f"{kk}={vv}" for kk, vv in d[k].items()))
        print()
