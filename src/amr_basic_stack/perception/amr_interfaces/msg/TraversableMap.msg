# Traversability 출력 → Planning (설계문서 §7.1)
#
# grid.data 의 값 규약 (nav_msgs/OccupancyGrid.data 는 int8[] 이다):
#     0   통과 가능
#   100   통과 불가
#    -1   미확인          (uint8 바이트로는 255 와 같은 값이다)
#
# 1~99 는 1단계에서 쓰지 않는다. cost 를 넣을 때를 위해 비워 둔다.
# 100 을 쓰는 이유: 소비자들이 occupied_thresh 0.65 / occupied_threshold 65
# 로 판정하므로 1 을 보내면 "1% 점유" = free 로 읽힌다.
std_msgs/Header          header

# 행 우선(row-major), row 0 = 최소 y. origin 은 셀 (0,0) 의 좌하단.
# 즉 PGM 이미지 배열과는 상하가 반대다.
nav_msgs/OccupancyGrid   grid

# 이 격자를 만든 z-band. height_frame 원점 기준이며 z 축은 수직 위쪽이다.
# camera optical 좌표계의 z(전방 깊이)가 아니다.
string                   height_frame
float32                  z_min
float32                  z_max

# 로봇 형상 팽창을 인지 측에서 적용했는지 (§7.1 "어느 단계에서 적용했는지 명시").
# 0.0 이면 적용하지 않은 것이며 Planning 이 자기 footprint 로 팽창해야 한다.
float32                  inflation_radius

# 미확인 셀을 무엇으로 메웠는지. 비트마스크.
uint8 UNKNOWN_FILL_NONE=0
uint8 UNKNOWN_FILL_PREVIOUS=1
uint8 UNKNOWN_FILL_BASE_MAP=2
uint8                    unknown_fill

# 이 격자에 반영된 관측 중 가장 오래된 것의 시각 (§6 유효성 판정용).
builtin_interfaces/Time  oldest_observation

# Minimum of the latest successfully transformed capture stamps of configured
# sensors (scan, and depth when enabled). Zero until every required sensor has
# supplied an observation. Unlike oldest_observation, this measures input freshness.
builtin_interfaces/Time sensor_watermark
