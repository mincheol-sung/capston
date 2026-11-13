"""Traversable map 의 값 규약과 병합 규칙.

여기는 ROS 에 의존하지 않는다. 노드와 테스트가 같은 함수를 쓴다.

값 규약 (층2 = wire = nav_msgs/OccupancyGrid.data, int8[]):
    0   통과 가능
  100   통과 불가
   -1   미확인

배열 순서는 OccupancyGrid 규약을 따른다 — row-major, row 0 = 최소 y,
origin 은 셀 (0,0) 의 좌하단. PGM 이미지 배열(row 0 = 최대 y)과 상하가 반대이며,
이미지 순서로 바꾸는 flip 은 소비자 경계에서 한 번만 한다.
"""

import numpy as np

FREE = 0
BLOCKED = 100
UNKNOWN = -1

# nav2 map_server 의 occupied_thresh 0.65, DistanceField 의 occupied_threshold 65
# 와 같은 경계다. 소비자 전부가 이 값으로 판정하므로 여기서도 같은 값을 쓴다.
OCCUPIED_THRESHOLD = 65

# 셀을 BLOCKED 으로 만든 근거. 비트마스크로 여러 센서를 함께 들 수 있다.
# 이걸 기록하는 이유: 2D 격자에서 값 100 하나만 보면 "왜 막혔는지" 를 알 수 없어
# 해제 증거를 센서별로 맞출 수 없다. LiDAR 평면(0.2178 m) 광선이 관통했다는
# 사실은 그 평면이 비었다는 증거일 뿐이고, 그보다 낮은 물체를 D455 가 밴드에서
# 본 것을 해제할 근거가 못 된다(설계문서 §6 "LiDAR 빔이 장애물 위를 지나갔다고
# 해서 낮은 장애물이 사라졌다고 판단하면 안 된다").
SOURCE_NONE = 0
SOURCE_LIDAR = 1
SOURCE_DEPTH = 2

_DROP_LIDAR = np.uint8(0xFF ^ SOURCE_LIDAR)
_DROP_DEPTH = np.uint8(0xFF ^ SOURCE_DEPTH)

UNKNOWN_FILL_NONE = 0
UNKNOWN_FILL_PREVIOUS = 1
UNKNOWN_FILL_BASE_MAP = 2


def classify_base_map(data, trust_free):
    """기준 2D 점유 격자(int8)를 3-state 로 변환한다.

    trust_free=False 이면 free 셀도 미확인으로 내린다. 2단계에서 D455 가
    들어와 traversability 정의가 '2D LiDAR 평면이 비어 있다' 보다 넓어지면
    기준 맵의 free 는 근거가 되지 못하므로 그때 False 로 뒤집는다.
    """
    base = np.asarray(data, dtype=np.int8)
    out = np.full(base.shape, UNKNOWN, dtype=np.int8)
    out[base >= OCCUPIED_THRESHOLD] = BLOCKED
    if trust_free:
        out[(base >= 0) & (base < OCCUPIED_THRESHOLD)] = FREE
    return out


def update_memory(memory, observed_at, observation, now, memory_seconds,
                  free_memory_seconds=0.0):
    """관측 기억을 한 주기 갱신한다.

    observation 에서 UNKNOWN 이 아닌 셀만 기억을 덮어쓰고 그 셀의 관측 시각을
    now 로 올린다.

    만료는 셀별 observed_at 으로 판정한다. 격자 전체에 타임스탬프 하나만 두면
    매 주기 기억을 다시 실어 나르면서 시각이 계속 갱신되어 만료가 영원히
    오지 않는다.

    BLOCKED 과 FREE 는 만료 시간이 다르다.

    * BLOCKED 은 memory_seconds 뒤에 UNKNOWN 으로 내린다. 동적 장애물이
      시야에서 사라졌을 때 영원히 남지 않게 하기 위함이며, 빈 공간으로
      확정하는 것이 아니라 "모르는 상태" 로 되돌리는 것이다(설계문서 §6).
    * FREE 는 free_memory_seconds 가 0 이하이면 만료시키지 않는다. 광선이
      관통했다는 증거는 시간이 지나도 사라지지 않으며, 이 값이 기준 맵의
      오래된 점유를 덮는 유일한 수단이다(§6 "재관측 후 계획용 지도에 별도
      해제 규칙"). 다시 막히면 그 셀에 hit 이 찍혀 BLOCKED 으로 덮인다.

    반환: (memory, observed_at) — 둘 다 새 배열.
    """
    observation = np.asarray(observation, dtype=np.int8)
    fresh = observation != UNKNOWN

    memory = np.where(fresh, observation, memory).astype(np.int8)
    observed_at = np.where(fresh, float(now), observed_at).astype(np.float64)

    age = float(now) - observed_at
    expired = (memory == BLOCKED) & (age > float(memory_seconds))
    if float(free_memory_seconds) > 0.0:
        expired |= (memory == FREE) & (age > float(free_memory_seconds))
    memory = np.where(expired, UNKNOWN, memory).astype(np.int8)
    return memory, observed_at


def update_memory_sourced(memory, source, observed_at,
                          lidar_hits, depth_hits, lidar_clear, depth_clear,
                          now, memory_seconds, free_memory_seconds=0.0):
    """센서별 근거를 따로 기억하고, 해제도 같은 센서의 증거로만 한다.

    update_memory 와 달리 3-state 하나를 받지 않고 네 개의 bool 마스크를 받는다.
    두 센서의 marking 과 clearing 이 서로를 지우지 않게 하려면 격자에 근거가
    남아 있어야 하기 때문이다.

    규칙:

    * hit 은 해당 비트를 세운다. clear 는 해당 비트만 내린다. 다른 센서의
      비트는 건드리지 않는다 — 이것이 낮은 장애물이 LiDAR 광선에 지워지는
      것을 막는 유일한 지점이다.
    * 같은 주기에 hit 과 clear 가 겹치면 hit 이 이긴다. 있는 것을 없다고
      하는 쪽이 위험하다.
    * 근거가 하나도 남지 않은 셀만 FREE 가 된다. 하나라도 남아 있으면
      BLOCKED 이다.
    * 이번 주기에 아무 증거도 없는 셀은 기억을 그대로 들고 간다. 관측이
      끊긴 것을 해제 근거로 쓰지 않는다(§6).

    만료는 update_memory 와 같은 셀별 observed_at 판정이되, **memory_seconds
    가 0 이하이면 BLOCKED 을 만료시키지 않는다**. free_memory_seconds 와 같은
    규약이다. 관측 증거로만 지우고 싶을 때 0 으로 둔다. BLOCKED 이 만료되면
    근거도 함께 버린다 — 남겨 두면 다음 주기에 곧바로 다시 BLOCKED 을
    주장한다.

    반환: (memory, source, observed_at) — 셋 다 새 배열.
    """
    memory = np.array(memory, dtype=np.int8, copy=True)
    source = np.array(source, dtype=np.uint8, copy=True)
    observed_at = np.array(observed_at, dtype=np.float64, copy=True)

    lidar_hits = np.asarray(lidar_hits, dtype=bool)
    depth_hits = np.asarray(depth_hits, dtype=bool)
    lidar_clear = np.asarray(lidar_clear, dtype=bool)
    depth_clear = np.asarray(depth_clear, dtype=bool)

    source[lidar_hits] |= np.uint8(SOURCE_LIDAR)
    source[depth_hits] |= np.uint8(SOURCE_DEPTH)
    source[lidar_clear & ~lidar_hits] &= _DROP_LIDAR
    source[depth_clear & ~depth_hits] &= _DROP_DEPTH

    evidence = lidar_hits | depth_hits | lidar_clear | depth_clear
    memory[evidence & (source != SOURCE_NONE)] = BLOCKED
    memory[evidence & (source == SOURCE_NONE)] = FREE
    observed_at[evidence] = float(now)

    age = float(now) - observed_at
    expired = np.zeros(memory.shape, dtype=bool)
    if float(memory_seconds) > 0.0:
        expired = (memory == BLOCKED) & (age > float(memory_seconds))
        source[expired] = SOURCE_NONE
    if float(free_memory_seconds) > 0.0:
        expired = expired | ((memory == FREE) & (age > float(free_memory_seconds)))
    memory[expired] = UNKNOWN
    return memory, source, observed_at


def compose(memory, base):
    """관측 기억을 기준 맵 위에 얹어 publish 할 격자를 만든다.

    우선순위는 기억 > 기준 맵이다. 기억이 UNKNOWN 인 셀만 기준 맵으로 메운다.
    기준 맵도 UNKNOWN 이면 UNKNOWN 으로 남는다 — 관측한 적 없는 공간을 빈
    공간으로 바꾸지 않는다(설계문서 §7.1).

    base 가 None 이면 기억을 그대로 쓴다.
    """
    memory = np.asarray(memory, dtype=np.int8)
    if base is None:
        return memory.copy()
    base = np.asarray(base, dtype=np.int8)
    return np.where(memory != UNKNOWN, memory, base).astype(np.int8)


def unknown_fill_flags(memory, base, composed):
    """어떤 출처를 실제로 썼는지 비트마스크로 돌려준다.

    rclpy 의 uint8 필드는 numpy 스칼라를 받지 않고 Python int 만 받는다.
    """
    flags = UNKNOWN_FILL_NONE
    if np.any(memory != UNKNOWN):
        flags |= UNKNOWN_FILL_PREVIOUS
    if base is not None and np.any((memory == UNKNOWN) & (composed != UNKNOWN)):
        flags |= UNKNOWN_FILL_BASE_MAP
    return int(flags)


def to_obstacle_mask(data, shape, unknown_is_obstacle):
    """Wire 격자(int8)를 소비자용 bool 장애물 마스크로 바꾼다.

    unknown_is_obstacle 은 소비자마다 다르다 — global planner 는 미확인을
    통과 허용(False), local planner 는 통과 불가(True)로 쓴다(설계문서 §4).

    반환 배열은 입력과 같은 OccupancyGrid 순서다. 이미지 순서를 쓰는
    소비자(Theta*)는 이 결과에 np.flipud 를 한 번 더 걸어야 한다.
    """
    grid = np.asarray(data, dtype=np.int8).reshape(shape)
    mask = grid >= OCCUPIED_THRESHOLD
    if unknown_is_obstacle:
        mask |= grid < 0
    return mask
