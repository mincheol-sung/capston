# Complete receding-horizon planar reference generated at header.stamp.
std_msgs/Header header

# Time interval between consecutive references in seconds.
float64 dt

# Ordered horizon X*[t] = [x, y, yaw, vx, vy, yaw_rate].
# references[i] targets header.stamp + (i + 1) * dt.
PlanarReference[] references
