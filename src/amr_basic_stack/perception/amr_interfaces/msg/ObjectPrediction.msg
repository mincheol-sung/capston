# Motion Predictor / EKF 출력: 미래 궤적
std_msgs/Header                     header
uint32                              track_id
int32                               class_id
geometry_msgs/PoseWithCovariance[]  predicted_poses
float32[]                           time_offsets
