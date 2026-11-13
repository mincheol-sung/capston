# One planar pose and body-velocity reference in a reference horizon.
std_msgs/Header header

# Desired planar pose in header.frame_id (normally odom).
float64 x
float64 y
float64 yaw

# Desired body-frame velocity at the reference pose.
float64 vx
float64 vy
float64 yaw_rate
