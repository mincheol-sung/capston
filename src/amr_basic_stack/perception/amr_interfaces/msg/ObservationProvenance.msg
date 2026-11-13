# Original sensor record; never rewritten when coordinates change epoch.
std_msgs/Header header
string sensor_id
# Stable within a recording: topic|frame|stamp_ns|detection index / cluster beam set.
string observation_id
# Original LaserScan.ranges indices, including after sector masking/wraparound.
uint32[] beam_indices
# Seconds from original header.stamp, same order as beam_indices.
# Empty when timing is unknown; NEVER infer these from scan_time / N.
float64[] beam_time_offsets
uint8 TIMING_UNKNOWN=0
uint8 TIMING_VERIFIED=1
uint8 timing_status
